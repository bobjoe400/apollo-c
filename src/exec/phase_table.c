/*
 * phase_table.c -- PHASE TABLE MAINTENANCE: PHASCHNG, 2PHSCHNG, NEWPHASE,
 *                  MR.KLEAN / P00KLEAN / V37KLEAN, NEWMODEX / NEWMODEA /
 *                  MMDSPLAY / SETUPDSP / CHECKMM.
 *
 * Source: PHASE_TABLE_MAINTENANCE.agc, Comanche055 pp. 1404-1413,
 * Luminary099 pp. 1294-1302; NEWPHASE from RESTART_TABLES.agc (C p.220,
 * L p.243); the KLEANs from FRESH_START_AND_RESTART.agc (C p.185, L p.213).
 * The two PHASE_TABLE_MAINTENANCE listings are the same program; Luminary
 * adds the entry PHSCHNGA (code word in A, used by V37RET), which is
 * agc_phase_change_var() with the code passed explicitly. The KLEANs
 * differ: Comanche's P00KLEAN kills group 1 and V37KLEAN does not;
 * Luminary's V37KLEAN kills group 1 (see agc_phase_change_program_klean).
 *
 * Each of the six restart groups n has the erasable pair -PHASEn, PHASEn
 * (they must stay complements; GOPROG3 checks this before restarting),
 * TBASEn (TIME1 when the phase was set, for timed task restarts), and the
 * variable restart point PHSNAMEn / PHSPRDTn. LONGBASE (TIME2,TIME1) is
 * shared by every LONGCALL restart.
 *
 * The code word after TC PHASCHNG (15 bits, octal):
 *   type A  TL0 00P PPP PPP GGG   fixed phase P of group G
 *   type B  TL1 DAP PPP PPP GGG   variable job + the fixed phase P
 *   type C  TL0 1AD XXX CJW GGG   variable job (J), task (W) or LONGCALL (C)
 * T: set TBASEG, L: set LONGBASE, D: a new priority / delta time is given,
 * A: a restart 2CADR is given. PHASEn receives bits 13-4 of the code word
 * shifted down 3 (MP BIT12), so the type bits stay visible to RESTARTS:
 *   type A  phase 0..127, type B  01000 + D 0400 + A 0200 + phase,
 *   type C  0400 + A 0200 + D 0100 + XXX 070 + CJW.
 */
#include "agc/agc_exec.h"
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "exec_internal.h"
#include "exec_pending.h"

#define PH_WORD15   077777u   /* 15-bit word; also -0                     */
#define PH_NEG0     077777u   /* -0 (NEG0) in -PHASEn                     */
#define PH_TIME1    037777u   /* TIME1 is the low 14 bits of the clock    */
#define PH_GROUP    07u       /* GGG                                      */
#define PH_TBIT     040000u   /* T: set TBASE                             */
#define PH_LBIT     020000u   /* L: set LONGBASE                          */
#define PH_TYPEB    010000u   /* bit 13: type B                           */
#define PH_TYPEC    004000u   /* bit 12: type C (when bit 13 is clear)    */
#define PH_ABIT     002000u   /* A: 2CADR given (types B and C)           */
#define PH_DBIT_B   004000u   /* D in type B (bit 12)                     */
#define PH_DBIT_C   001000u   /* D in type C (bit 10)                     */
#define PH_PMASK    017770u   /* OCT17770, bits 13-4                      */
#define PH_PSHIFT   3         /* MP BIT12 = shift right 3                 */
#define PH_PRIO30   ((agc_prio_t)030)

typedef enum ph_type { PH_TYPE_A = 0, PH_TYPE_B, PH_TYPE_C } ph_type_t;

typedef struct ph_code {
    int       group;    /* 1..6 (GGG)                                  */
    uint16_t  tempp;    /* TEMPP: what goes into PHASEn                */
    ph_type_t type;
    bool      tbit;
    bool      lbit;
    bool      dbit;
} ph_code_t;

/* Group n is index n; index 0 is unused. A fresh AGC's DOFSTART kills
 * every group, so the static image starts with all groups killed. */
static uint16_t phase_w[AGC_RESTART_GROUPS + 1];          /* PHASEn      */
static uint16_t nphase_w[AGC_RESTART_GROUPS + 1] = {      /* -PHASEn     */
    PH_NEG0, PH_NEG0, PH_NEG0, PH_NEG0, PH_NEG0, PH_NEG0, PH_NEG0 };
static uint16_t tbase[AGC_RESTART_GROUPS + 1];  /* TIME1 at set, cs     */
static phase_var_store_t var[AGC_RESTART_GROUPS + 1]; /* PHSNAMEn/PRDTn */
static uint32_t longbase;                       /* TIME2,TIME1 at set cs */

static bool group_ok(int group)
{
    (void)AGC_ASSERT(phase_w[0] == 0U);
    (void)AGC_ASSERT(nphase_w[0] == PH_NEG0);
    return group >= 1 && group <= AGC_RESTART_GROUPS;
}

/* DXCH -PHASEn with (-phase, phase): both words of the pair at once. */
void phase_set_fixed(int group, uint16_t phase)
{
    if (!AGC_ASSERT(group_ok(group)) ||
        !AGC_ASSERT(phase <= PH_WORD15)) {
        return;
    }
    phase_w[group] = phase;
    nphase_w[group] = (uint16_t)(~phase & PH_WORD15);
    (void)AGC_ASSERT(phase_group_agrees(group));
}

/* EXTEND / DCA NEG0 / DXCH -PHASEn: -PHASEn = -0, PHASEn = +0.        */
void phase_kill(int group)
{
    if (!AGC_ASSERT(group_ok(group))) {
        return;
    }
    nphase_w[group] = PH_NEG0;
    phase_w[group] = 0;
    (void)AGC_ASSERT(phase_group_agrees(group));
}

agc_status_t agc_phase_kill(int group)
{
    if (!AGC_ASSERT(group_ok(group))) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    phase_kill(group);
    agc_relint();
    (void)AGC_ASSERT(phase_w[group] == 0U);
    return AGC_STATUS_OK;
}

/* CS TIME1 / TS TBASEn. DEVIATION: the AGC stores -TIME1 and RESTARTS
 * adds it back; TIME1 itself is stored here (same elapsed time).      */
static void set_tbase(int group)
{
    if (!AGC_ASSERT(group_ok(group))) {
        return;
    }
    tbase[group] = (uint16_t)(agc_time_now_cs() & PH_TIME1);
    (void)AGC_ASSERT(tbase[group] <= PH_TIME1);
}

/* PHSCHNG2 decoding (C p.1410 / L p.1299): TEMPG, TEMPP, TEMPSW.      */
static bool decode(uint16_t code, ph_code_t *out)
{
    if (!AGC_ASSERT(out != 0) ||
        !AGC_ASSERT(code <= PH_WORD15)) {
        return false;
    }
    out->group = (int)(code & PH_GROUP);
    out->tempp = (uint16_t)((code & PH_PMASK) >> PH_PSHIFT);
    out->tbit = (code & PH_TBIT) != 0U;
    out->lbit = (code & PH_LBIT) != 0U;
    if ((code & PH_TYPEB) != 0U) {
        out->type = PH_TYPE_B;                    /* CHECKB            */
        out->dbit = (code & PH_DBIT_B) != 0U;
    } else if ((code & PH_TYPEC) != 0U) {
        out->type = PH_TYPE_C;
        out->dbit = (code & PH_DBIT_C) != 0U;
    } else {
        out->type = PH_TYPE_A;
        out->dbit = false;
    }
    return AGC_ASSERT(group_ok(out->group));
}

/* The restart kind a type C phase names by its CJW bits, as RESTARTS
 * decodes it ((CJW) - 2: > 0 LONGCALL, -0 job, < 0 task).             */
static agc_restart_kind_t cjw_kind(uint16_t tempp)
{
    int cjw = (int)(tempp & PH_GROUP) - 2;
    agc_restart_kind_t k = AGC_RESTART_TASK;
    if (cjw > 0) {
        k = AGC_RESTART_LONGCALL;
    } else if (cjw == 0) {
        k = AGC_RESTART_JOB;
    } else {
        /* W: waitlist task */
    }
    (void)AGC_ASSERT(cjw >= -2 && cjw <= 5);
    return k;
}

/* GETPRIO / OLDPRIO / GETNEWNM / CON2 (C pp.1410-1412 / L pp.1299-1301):
 * store the variable restart point of a type B or C phase. With D clear
 * the old PHSPRDTn is kept; v == NULL keeps the old PHSNAMEn as well.
 * DEVIATION: the AGC takes the restart point from the return address
 * when A is clear; C cannot name a return address, so the caller always
 * names it in v (and an indirect delta time is resolved by the caller,
 * at the time of the phase change, into v->prio_or_dt / long_dt_cs).   */
static void store_var(const ph_code_t *c, const agc_phase_var_t *v)
{
    if (!AGC_ASSERT(c != 0) ||
        !AGC_ASSERT(c->type != PH_TYPE_A)) {
        return;
    }
    phase_var_store_t *s = &var[c->group];
    agc_restart_kind_t k = (c->type == PH_TYPE_B) ? AGC_RESTART_JOB
                                                  : cjw_kind(c->tempp);
    if (v == 0) {
        return;
    }
    (void)AGC_ASSERT(v->kind == k);
    s->kind = k;
    s->job = v->job;
    s->task = v->task;
    if (c->dbit) {
        s->prdt = v->prio_or_dt;
        s->immediate = v->immediate;
        s->long_dt_cs = v->long_dt_cs;
    }
}

/* BELOW1 for 2PHSCHNG: the type A phase of the second group.          */
typedef struct ph_second {
    bool     present;
    int      group;     /* TEMPG2 / 2                                  */
    uint16_t phase;     /* TEMPP2                                      */
    bool     tbit;      /* TEMPSW2 = BIT15 of the first code word      */
} ph_second_t;

/* BELOW1 .. BELOW4 (C pp.1411-1412 / L pp.1300-1301).                 */
static void below1_4(const ph_code_t *c, const ph_second_t *two)
{
    if (!AGC_ASSERT(c != 0) ||
        !AGC_ASSERT(two != 0)) {
        return;
    }
    if (two->present && AGC_ASSERT(group_ok(two->group))) {
        phase_set_fixed(two->group, two->phase);  /* DXCH -PHASE(G2)   */
        if (two->tbit) {
            set_tbase(two->group);
        }
    }
    if (c->tbit) {
        set_tbase(c->group);                      /* BELOW2 -> TBASE   */
    }
    if (c->lbit) {
        longbase = agc_time_now_cs();             /* BELOW3: DCA TIME2 */
    }
    phase_set_fixed(c->group, c->tempp);          /* BELOW4: -PHASE    */
    (void)AGC_ASSERT(phase_w[c->group] == c->tempp);
}

static agc_status_t phaschng_common(uint16_t code, const agc_phase_var_t *v,
                                    const ph_second_t *two)
{
    ph_code_t c;
    if (!decode(code, &c)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    if (c.type != PH_TYPE_A) {
        store_var(&c, v);
    }
    below1_4(&c, two);
    agc_relint();
    return AGC_STATUS_OK;
}

/* PHASCHNG (C p.1409 / L p.1298), fixed form: `group` must be the GGG of
 * the code word. A type B/C code given here keeps the stored restart
 * point and priority (as if D = 0 and the old 2CADR were still right). */
agc_status_t agc_phase_change(int group, uint16_t phase_code)
{
    if (!AGC_ASSERT(group_ok(group)) ||
        !AGC_ASSERT((int)(phase_code & PH_GROUP) == group)) {
        return AGC_STATUS_BAD_ARG;
    }
    ph_second_t none = { false, 0, 0, false };
    return phaschng_common(phase_code, 0, &none);
}

/* PHASCHNG / PHSCHNGA type B or C with the variable restart point.    */
agc_status_t agc_phase_change_var(uint16_t code, const agc_phase_var_t *v)
{
    if (!AGC_ASSERT(v != 0) ||
        !AGC_ASSERT(v->kind != AGC_RESTART_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    ph_second_t none = { false, 0, 0, false };
    return phaschng_common(code, v, &none);
}

/* 2PHSCHNG (C p.1409 / L p.1298): code_a (type A only; its L bit is
 * disregarded) for one group, code_b (A, B or C) for another. v may be
 * NULL when code_b is type A.                                          */
agc_status_t agc_phase_change2(uint16_t code_a, uint16_t code_b,
                               const agc_phase_var_t *v)
{
    ph_code_t a;
    if (!decode(code_a, &a) || !AGC_ASSERT(a.type == PH_TYPE_A)) {
        return AGC_STATUS_BAD_ARG;
    }
    ph_second_t two = { true, a.group, a.tempp, a.tbit };
    return phaschng_common(code_b, v, &two);
}

/* NEWPHASE (RESTART_TABLES C p.221 / L p.243): quick fixed phase change
 * of `group` to `phase`, optionally setting TBASE (the AGC passes the
 * phase negative in A to ask for that).                                */
agc_status_t agc_phase_newphase(int group, uint16_t phase, bool set_tb)
{
    if (!AGC_ASSERT(group_ok(group)) ||
        !AGC_ASSERT(phase <= 0177U)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    if (set_tb) {
        set_tbase(group);                         /* NUFAZ+10          */
    }
    phase_set_fixed(group, phase);
    agc_relint();
    return AGC_STATUS_OK;
}

/* Contract helper: make `group` restart `job` as a FINDVAC job of
 * priority `prio`, i.e. TC PHASCHNG / OCT 07020+G / OCT prio / 2CADR job
 * (type C, A, D, J).                                                   */
agc_status_t agc_phase_new(int group, agc_job_id_t job, agc_prio_t prio)
{
    if (!AGC_ASSERT(group_ok(group)) ||
        !AGC_ASSERT(job != AGC_JOB_NONE) ||
        !AGC_ASSERT(prio >= 1 && prio <= 037)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_phase_var_t v = { AGC_RESTART_JOB, job, AGC_TASK_NONE,
                          (int16_t)prio, false, 0U };
    return agc_phase_change_var((uint16_t)(07020U | (uint16_t)group), &v);
}

uint16_t agc_phase_get(int group)
{
    if (!AGC_ASSERT(group_ok(group))) {
        return 0;
    }
    (void)AGC_ASSERT(phase_w[group] <= PH_WORD15);
    return phase_w[group];
}

uint16_t phase_word(int group)
{
    (void)AGC_ASSERT(group_ok(group));
    return agc_phase_get(group);
}

/* PCLOOP test (FRESH_START C p.188 / L p.218): -PHASEn RXOR PHASEn must
 * be -0, i.e. the two words are complements.                           */
bool phase_group_agrees(int group)
{
    if (!AGC_ASSERT(group_ok(group))) {
        return false;
    }
    (void)AGC_ASSERT(nphase_w[group] <= PH_WORD15);
    return (uint16_t)(nphase_w[group] ^ phase_w[group]) == PH_NEG0;
}

agc_status_t phase_var_get(int group, phase_var_store_t *out)
{
    if (!AGC_ASSERT(group_ok(group)) ||
        !AGC_ASSERT(out != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    *out = var[group];
    return AGC_STATUS_OK;
}

uint16_t phase_tbase(int group)
{
    if (!AGC_ASSERT(group_ok(group))) {
        return 0;
    }
    (void)AGC_ASSERT(tbase[group] <= PH_TIME1);
    return tbase[group];
}

uint32_t phase_longbase(void)
{
    (void)AGC_ASSERT(longbase < AGC_TIME_MODULUS_CS);
    (void)AGC_ASSERT(phase_w[0] == 0U);
    return longbase;
}

/* Test hook: corrupt -PHASEn (as a restart between the two stores of a
 * PHASCHNG would leave it) so the GOPROG3 table check can be exercised. */
void phase_debug_set_negative(int group, uint16_t neg_word)
{
    if (!AGC_ASSERT(group_ok(group)) ||
        !AGC_ASSERT(neg_word <= PH_WORD15)) {
        return;
    }
    nphase_w[group] = neg_word;
}

/* V37KLEAN (FRESH_START C p.185 / L p.213): kill groups 3, 5, 6, and in
 * Luminary also group 1.                                               */
agc_status_t agc_phase_change_program_klean(void)
{
    agc_inhint();
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        phase_kill(1);
    }
    phase_kill(3);
    phase_kill(5);
    phase_kill(6);
    agc_relint();
    (void)AGC_ASSERT(phase_w[3] == 0U && phase_w[5] == 0U);
    return AGC_STATUS_OK;
}

/* P00KLEAN: kill group 4 (Comanche also group 1), then V37KLEAN.       */
agc_status_t agc_phase_idle_klean(void)
{
    agc_inhint();
    phase_kill(4);
    if (agc_state_vehicle() == AGC_VEHICLE_CSM) {
        phase_kill(1);
    }
    agc_relint();
    (void)AGC_ASSERT(phase_w[4] == 0U);
    return agc_phase_change_program_klean();
}

/* MR.KLEAN: kill group 2, then P00KLEAN -- every group is inactive.    */
agc_status_t agc_phase_mr_klean(void)
{
    agc_inhint();
    phase_kill(2);
    agc_relint();
    agc_status_t st = agc_phase_idle_klean();
    (void)AGC_ASSERT(phase_w[1] == 0U && phase_w[6] == 0U);
    return st;
}

/* MMDSPLAY -> SETUPDSP (C p.1404 / L p.1294): NOVAC PRIO30 DSPMMJOB to
 * show MODREG in the PROG lights.                                      */
agc_status_t fresh_mmdsplay(void)
{
    agc_inhint();
    agc_status_t st = agc_exec_novac(PH_PRIO30, AGC_JOB_DSPMMJB);
    agc_relint();
    (void)AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BAILOUT);
    return st;
}

/* NEWMODEX / NEWMODEA (C p.1404 / L p.1294): store the major mode in
 * MODREG and display it.                                               */
agc_status_t agc_phase_newmode(agc_sp_t mode)
{
    if (!AGC_ASSERT(mode >= EXEC_MODREG_NEG0 && mode <= 0177)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_erase_set_modreg(mode);                   /* TS MODREG         */
    return fresh_mmdsplay();
}

/* CHECKMM (C p.1404 / L p.1294): does MODREG equal `mode`?             */
bool agc_phase_checkmm(agc_sp_t mode)
{
    (void)AGC_ASSERT(mode >= EXEC_MODREG_NEG0 && mode <= 0177);
    agc_sp_t mm = agc_erase_modreg();
    (void)AGC_ASSERT(mm >= EXEC_MODREG_NEG0);
    return mm == mode;
}
