/*
 * agc_erasable.c -- storage and accessors for the widely shared erasables.
 *
 * One static object per AGC erasable (rule 6: no extern globals). Generated
 * from the table in docs/modules/state-flags.md; every accessor is a plain
 * get/set with an input range check. Scale factors are in agc_state.h.
 */
#include <stdbool.h>
#include <stdint.h>
#include "agc/agc_assert.h"
#include "agc/agc_state.h"

#define AGC_WORD15_MAX 077777u   /* a raw 15-bit AGC word */

static bool sane(void)
{
    /* The storage below is statically initialised; nothing to verify at
     * run time except that the vehicle selector is a legal value. */
    const agc_vehicle_t v = agc_state_vehicle();
    return (v == AGC_VEHICLE_CSM) || (v == AGC_VEHICLE_LM);
}

static bool dp_ok(agc_dp_t v)
{
    return (v >= AGC_DP_MIN) && (v <= AGC_DP_MAX);
}

static bool sp_ok(agc_sp_t v)
{
    return (v >= AGC_SP_MIN) && (v <= AGC_SP_MAX);
}

static bool tp_ok(agc_tp_t v)
{
    return (v >= AGC_TP_MIN) && (v <= AGC_TP_MAX);
}

static bool w15_ok(uint16_t v)
{
    return v <= AGC_WORD15_MAX;
}

static bool vec_ok(const agc_vec_t *v)
{
    bool ok = true;
    for (int i = 0; i < AGC_VEC_N; i++) {
        ok = ok && dp_ok(v->c[i]);
    }
    return ok;
}

static bool mat_ok(const agc_mat_t *m)
{
    bool ok = true;
    for (int i = 0; i < AGC_VEC_N; i++) {
        ok = ok && vec_ok(&m->row[i]);
    }
    return ok;
}

/* ---- storage ---- */
static agc_sp_t g_modreg;           /* MODREG */
static agc_dp_t g_piptime;
static agc_dp_t g_tig;
static agc_dp_t g_ttogo;
static agc_dp_t g_tgo;
static agc_dp_t g_ttpi;
static agc_dp_t g_tevent;
static agc_dp_t g_tetcsm;
static agc_dp_t g_tetlem;
static agc_vec_t g_rn;
static agc_vec_t g_vn;
static agc_vec_t g_rrectcsm;
static agc_vec_t g_vrectcsm;
static agc_vec_t g_rrectlem;
static agc_vec_t g_vrectlem;
static agc_vec_t g_delv;
static agc_vec_t g_delvslv;
static agc_vec_t g_vgtig;
static agc_mat_t g_refsmmat;
static agc_tp_t g_tephem;
static agc_sp_t g_csmmass;
static agc_sp_t g_lemmass;
static agc_sp_t g_pipax;
static agc_sp_t g_pipay;
static agc_sp_t g_pipaz;
static uint16_t g_cdux;
static uint16_t g_cduy;
static uint16_t g_cduz;
static uint16_t g_cdut;
static uint16_t g_cdus;
static uint16_t g_markstat;
static uint16_t g_imodes30;
static uint16_t g_imodes33;
static uint16_t g_dapdatr1;
static uint16_t g_dapdatr2;
static uint16_t g_dsptab[12];
static uint16_t g_failreg[3];
static uint16_t g_thetad[3];
agc_sp_t agc_erase_modreg(void)
{
    if (!AGC_ASSERT(sp_ok(g_modreg))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_modreg;
}

void agc_erase_set_modreg(agc_sp_t v)
{
    if (!AGC_ASSERT(sp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_modreg = v;
}

agc_dp_t agc_erase_piptime(void)
{
    if (!AGC_ASSERT(dp_ok(g_piptime))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_piptime;
}

void agc_erase_set_piptime(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_piptime = v;
}

agc_dp_t agc_erase_tig(void)
{
    if (!AGC_ASSERT(dp_ok(g_tig))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_tig;
}

void agc_erase_set_tig(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_tig = v;
}

agc_dp_t agc_erase_ttogo(void)
{
    if (!AGC_ASSERT(dp_ok(g_ttogo))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_ttogo;
}

void agc_erase_set_ttogo(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_ttogo = v;
}

agc_dp_t agc_erase_tgo(void)
{
    if (!AGC_ASSERT(dp_ok(g_tgo))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_tgo;
}

void agc_erase_set_tgo(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_tgo = v;
}

agc_dp_t agc_erase_ttpi(void)
{
    if (!AGC_ASSERT(dp_ok(g_ttpi))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_ttpi;
}

void agc_erase_set_ttpi(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_ttpi = v;
}

agc_dp_t agc_erase_tevent(void)
{
    if (!AGC_ASSERT(dp_ok(g_tevent))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_tevent;
}

void agc_erase_set_tevent(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_tevent = v;
}

agc_dp_t agc_erase_tetcsm(void)
{
    if (!AGC_ASSERT(dp_ok(g_tetcsm))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_tetcsm;
}

void agc_erase_set_tetcsm(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_tetcsm = v;
}

agc_dp_t agc_erase_tetlem(void)
{
    if (!AGC_ASSERT(dp_ok(g_tetlem))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_tetlem;
}

void agc_erase_set_tetlem(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_tetlem = v;
}

agc_vec_t agc_erase_rn(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_rn))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_rn;
}

void agc_erase_set_rn(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_rn = v;
}

agc_vec_t agc_erase_vn(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_vn))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_vn;
}

void agc_erase_set_vn(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_vn = v;
}

agc_vec_t agc_erase_rrectcsm(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_rrectcsm))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_rrectcsm;
}

void agc_erase_set_rrectcsm(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_rrectcsm = v;
}

agc_vec_t agc_erase_vrectcsm(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_vrectcsm))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_vrectcsm;
}

void agc_erase_set_vrectcsm(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_vrectcsm = v;
}

agc_vec_t agc_erase_rrectlem(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_rrectlem))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_rrectlem;
}

void agc_erase_set_rrectlem(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_rrectlem = v;
}

agc_vec_t agc_erase_vrectlem(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_vrectlem))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_vrectlem;
}

void agc_erase_set_vrectlem(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_vrectlem = v;
}

agc_vec_t agc_erase_delv(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_delv))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_delv;
}

void agc_erase_set_delv(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_delv = v;
}

agc_vec_t agc_erase_delvslv(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_delvslv))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_delvslv;
}

void agc_erase_set_delvslv(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_delvslv = v;
}

agc_vec_t agc_erase_vgtig(void)
{
    agc_vec_t zero = {{0, 0, 0}};
    if (!AGC_ASSERT(vec_ok(&g_vgtig))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_vgtig;
}

void agc_erase_set_vgtig(agc_vec_t v)
{
    if (!AGC_ASSERT(vec_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_vgtig = v;
}

agc_mat_t agc_erase_refsmmat(void)
{
    agc_mat_t zero = {{{{0, 0, 0}}, {{0, 0, 0}}, {{0, 0, 0}}}};
    if (!AGC_ASSERT(mat_ok(&g_refsmmat))) {
        return zero;
    }
    if (!AGC_ASSERT(sane())) {
        return zero;
    }
    return g_refsmmat;
}

void agc_erase_set_refsmmat(agc_mat_t v)
{
    if (!AGC_ASSERT(mat_ok(&v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_refsmmat = v;
}

agc_tp_t agc_erase_tephem(void)
{
    if (!AGC_ASSERT(tp_ok(g_tephem))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_tephem;
}

void agc_erase_set_tephem(agc_tp_t v)
{
    if (!AGC_ASSERT(tp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_tephem = v;
}

agc_sp_t agc_erase_csmmass(void)
{
    if (!AGC_ASSERT(sp_ok(g_csmmass))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_csmmass;
}

void agc_erase_set_csmmass(agc_sp_t v)
{
    if (!AGC_ASSERT(sp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_csmmass = v;
}

agc_sp_t agc_erase_lemmass(void)
{
    if (!AGC_ASSERT(sp_ok(g_lemmass))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_lemmass;
}

void agc_erase_set_lemmass(agc_sp_t v)
{
    if (!AGC_ASSERT(sp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_lemmass = v;
}

agc_sp_t agc_erase_pipax(void)
{
    if (!AGC_ASSERT(sp_ok(g_pipax))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_pipax;
}

void agc_erase_set_pipax(agc_sp_t v)
{
    if (!AGC_ASSERT(sp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_pipax = v;
}

agc_sp_t agc_erase_pipay(void)
{
    if (!AGC_ASSERT(sp_ok(g_pipay))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_pipay;
}

void agc_erase_set_pipay(agc_sp_t v)
{
    if (!AGC_ASSERT(sp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_pipay = v;
}

agc_sp_t agc_erase_pipaz(void)
{
    if (!AGC_ASSERT(sp_ok(g_pipaz))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_pipaz;
}

void agc_erase_set_pipaz(agc_sp_t v)
{
    if (!AGC_ASSERT(sp_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_pipaz = v;
}

uint16_t agc_erase_cdux(void)
{
    if (!AGC_ASSERT(w15_ok(g_cdux))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_cdux;
}

void agc_erase_set_cdux(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_cdux = v;
}

uint16_t agc_erase_cduy(void)
{
    if (!AGC_ASSERT(w15_ok(g_cduy))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_cduy;
}

void agc_erase_set_cduy(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_cduy = v;
}

uint16_t agc_erase_cduz(void)
{
    if (!AGC_ASSERT(w15_ok(g_cduz))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_cduz;
}

void agc_erase_set_cduz(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_cduz = v;
}

uint16_t agc_erase_cdut(void)
{
    if (!AGC_ASSERT(w15_ok(g_cdut))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_cdut;
}

void agc_erase_set_cdut(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_cdut = v;
}

uint16_t agc_erase_cdus(void)
{
    if (!AGC_ASSERT(w15_ok(g_cdus))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_cdus;
}

void agc_erase_set_cdus(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_cdus = v;
}

uint16_t agc_erase_markstat(void)
{
    if (!AGC_ASSERT(w15_ok(g_markstat))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_markstat;
}

void agc_erase_set_markstat(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_markstat = v;
}

uint16_t agc_erase_imodes30(void)
{
    if (!AGC_ASSERT(w15_ok(g_imodes30))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_imodes30;
}

void agc_erase_set_imodes30(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_imodes30 = v;
}

uint16_t agc_erase_imodes33(void)
{
    if (!AGC_ASSERT(w15_ok(g_imodes33))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_imodes33;
}

void agc_erase_set_imodes33(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_imodes33 = v;
}

uint16_t agc_erase_dapdatr1(void)
{
    if (!AGC_ASSERT(w15_ok(g_dapdatr1))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_dapdatr1;
}

void agc_erase_set_dapdatr1(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_dapdatr1 = v;
}

uint16_t agc_erase_dapdatr2(void)
{
    if (!AGC_ASSERT(w15_ok(g_dapdatr2))) {
        return 0;
    }
    if (!AGC_ASSERT(sane())) {
        return 0;
    }
    return g_dapdatr2;
}

void agc_erase_set_dapdatr2(uint16_t v)
{
    if (!AGC_ASSERT(w15_ok(v))) {
        return;
    }
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_dapdatr2 = v;
}

uint16_t agc_erase_dsptab(int index)
{
    if (!AGC_ASSERT((index >= 0) && (index < 12))) {
        return 0;
    }
    if (!AGC_ASSERT(w15_ok(g_dsptab[index]))) {
        return 0;
    }
    return g_dsptab[index];
}

agc_status_t agc_erase_set_dsptab(int index, uint16_t v)
{
    if (!AGC_ASSERT((index >= 0) && (index < 12))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(w15_ok(v))) {
        return AGC_STATUS_BAD_ARG;
    }
    g_dsptab[index] = v;
    return AGC_STATUS_OK;
}

uint16_t agc_erase_failreg(int index)
{
    if (!AGC_ASSERT((index >= 0) && (index < 3))) {
        return 0;
    }
    if (!AGC_ASSERT(w15_ok(g_failreg[index]))) {
        return 0;
    }
    return g_failreg[index];
}

agc_status_t agc_erase_set_failreg(int index, uint16_t v)
{
    if (!AGC_ASSERT((index >= 0) && (index < 3))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(w15_ok(v))) {
        return AGC_STATUS_BAD_ARG;
    }
    g_failreg[index] = v;
    return AGC_STATUS_OK;
}

uint16_t agc_erase_thetad(int index)
{
    if (!AGC_ASSERT((index >= 0) && (index < 3))) {
        return 0;
    }
    if (!AGC_ASSERT(w15_ok(g_thetad[index]))) {
        return 0;
    }
    return g_thetad[index];
}

agc_status_t agc_erase_set_thetad(int index, uint16_t v)
{
    if (!AGC_ASSERT((index >= 0) && (index < 3))) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(w15_ok(v))) {
        return AGC_STATUS_BAD_ARG;
    }
    g_thetad[index] = v;
    return AGC_STATUS_OK;
}

void agc_erase_reset(void)
{
    if (!AGC_ASSERT(sane())) {
        return;
    }
    g_modreg = 0;
    g_piptime = 0;
    g_tig = 0;
    g_ttogo = 0;
    g_tgo = 0;
    g_ttpi = 0;
    g_tevent = 0;
    g_tetcsm = 0;
    g_tetlem = 0;
    g_rn = (agc_vec_t){{0, 0, 0}};
    g_vn = (agc_vec_t){{0, 0, 0}};
    g_rrectcsm = (agc_vec_t){{0, 0, 0}};
    g_vrectcsm = (agc_vec_t){{0, 0, 0}};
    g_rrectlem = (agc_vec_t){{0, 0, 0}};
    g_vrectlem = (agc_vec_t){{0, 0, 0}};
    g_delv = (agc_vec_t){{0, 0, 0}};
    g_delvslv = (agc_vec_t){{0, 0, 0}};
    g_vgtig = (agc_vec_t){{0, 0, 0}};
    for (int i = 0; i < AGC_VEC_N; i++) {
        g_refsmmat.row[i] = (agc_vec_t){{0, 0, 0}};
    }
    g_tephem = 0;
    g_csmmass = 0;
    g_lemmass = 0;
    g_pipax = 0;
    g_pipay = 0;
    g_pipaz = 0;
    g_cdux = 0;
    g_cduy = 0;
    g_cduz = 0;
    g_cdut = 0;
    g_cdus = 0;
    g_markstat = 0;
    g_imodes30 = 0;
    g_imodes33 = 0;
    g_dapdatr1 = 0;
    g_dapdatr2 = 0;
    for (int i = 0; i < 12; i++) {
        g_dsptab[i] = 0;
    }
    for (int i = 0; i < 3; i++) {
        g_failreg[i] = 0;
    }
    for (int i = 0; i < 3; i++) {
        g_thetad[i] = 0;
    }
}
