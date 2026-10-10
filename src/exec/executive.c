/*
 * executive.c -- the AGC cooperative EXECUTIVE.
 *
 * Source: EXECUTIVE.agc, Comanche055 pp. 1208-1220, Luminary099 pp. 1103-1116.
 * The two listings are the same program except:
 *   - core sets: Comanche NO.CORES = DEC 6 -> 7 core sets; Luminary
 *     NO.CORES = DEC 7 -> 8 core sets (EJSCAN also tests PRIORITY +84D);
 *   - overflow aborts: Comanche TC BAILOUT, Luminary TC BAILOUT1 (which also
 *     records the caller's address in ALMCADR). Same codes 1201 / 1202.
 *
 * ---------------------------------------------------------------------
 * How the AGC executive is modelled
 * ---------------------------------------------------------------------
 * The AGC keeps 7 (CSM) or 8 (LM) "core sets" of 12 words: MPAC (7), LOC,
 * BANKSET, PUSHLOC and PRIORITY. Core set 0 always belongs to the job that
 * is running. PRIORITY holds the priority in bits 10-14 and, in its low 9
 * bits, the address of the job's VAC area (FINDVAC) or the fixed value
 * LOC(MPAC)-36D (NOVAC). It is -0 for a free core set and negative for a
 * sleeping job. Five 43-word VAC areas are claimed through VAC1USE..VAC5USE.
 * NEWJOB tells the running job that a higher-priority job is waiting:
 * +0 = keep running core set 0, +k = change to core set k, -0 = idle
 * (DUMMYJOB). An AGC job polls NEWJOB (the interpreter does it between
 * instructions; basic code at chosen points) and calls CHANG1 / CHANG2,
 * which swaps core set 0 with core set NEWJOB (CHANJOB).
 *
 * In C a job is a function body (agc_job_<name>(ctx)) that runs to
 * completion or to an explicit sleep/yield. agc_exec_run_one() does what
 * ADVAN / NUDIRECT / NUCHANG2 / CHANJOB do:
 *   1. if NEWJOB > 0 it performs CHANJOB (the pre-emption point): core set
 *      0 and core set NEWJOB are swapped exactly as in the AGC, so the
 *      core-set positions, scan order and tie-breaks stay faithful;
 *   2. it dispatches the job in core set 0 at LOC;
 *   3. afterwards it does what the job's exit did in the AGC: ENDOFJOB
 *      (free the VAC area and core set, EJSCAN for the highest active
 *      priority, CHANJOB or DUMMYJOB), JOBSLEEP (EJSCAN, the sleeper stays
 *      in its core set with a negative priority) or CHANG1 (agc_exec_job_
 *      yield: the job stays active with LOC = its continuation and is
 *      swapped out if NEWJOB is set).
 * The highest-priority ready job therefore always runs next, and equal
 * priorities are broken as the AGC breaks them (the low 9 bits of the
 * priority word, then core-set scan order).
 *
 * What is preserved: priorities and the priority word ordering, NEWJOB,
 * core-set positions and CHANJOB swaps, NOVAC/FINDVAC semantics, VAC
 * claiming order, 1201/1202 overflow, sleep/wake by LOC, PRIOCHNG, the
 * activity light (channel 11 bit 2).
 * What is lost: pre-emption in the middle of a job body. In the AGC a job
 * can lose the processor at any interpreter instruction or CHANG1 call; in
 * C only at the boundary of a job function or after agc_exec_job_yield().
 * A long job body therefore delays a newly created higher-priority job
 * until it returns. MPAC, OVFIND, PUSHLOC and the VAC contents are not
 * saved by the executive: a C job keeps its working data in locals (lost
 * at a sleep/yield, so a job that continues later stores what it needs in
 * module erasables) -- the VAC area is only accounted for, not used.
 */
#include "agc/agc_exec.h"
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "exec_internal.h"

/* Priority word layout: priority number in bits 10-14 (PRIOnn = nn000
 * octal), low 9 bits = VAC address or LOC(MPAC)-36D for NOVAC. The C
 * model replaces the address by its rank: NOVAC (FAKEPRET, an address in
 * the core-set area) is below every VAC area, and VAC1 < VAC2 < ... < VAC5
 * in erasable, which is all the comparisons in EJ1 / SETLOC depend on.   */
#define EXEC_PRIO_SHIFT   9
#define EXEC_PRIO_MAX     ((agc_prio_t)037)   /* PRIO37                  */
#define EXEC_NOVAC_RANK   0                   /* FAKEPRET                */
#define EXEC_NEWJOB_IDLE  (-1)                /* NEWJOB = -0 (DUMMYJOB)  */

typedef struct core_set {
    agc_core_state_t state;   /* sign of PRIORITY                        */
    agc_prio_t       prio;    /* PRIORITY bits 10-14                     */
    int              vac;     /* -1 NOVAC, else VAC area 0..4            */
    agc_job_id_t     loc;     /* LOC (the job's 2CADR)                   */
    agc_sp_t         arg;     /* MPAC as loaded by the creator           */
} core_set_t;

static core_set_t core[AGC_EXEC_MAX_JOBS];      /* PRIORITY, LOC, ... */
static bool       vac_used[AGC_EXEC_MAX_VACS];  /* VACnUSE = +0 in use */
static int        newjob = EXEC_NEWJOB_IDLE;    /* NEWJOB             */
static int        locctr = -1;                  /* LOCCTR             */
static bool       yielded;                      /* CHANG1 this pass   */
static bool       in_job;                       /* job body running   */
static uint16_t   generation;                   /* STARTSB2 count     */

/* NO.CORES + 1 for the vehicle being modelled. */
int agc_exec_core_sets(void)
{
    int n = AGC_EXEC_CORE_SETS_CSM;
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        n = AGC_EXEC_CORE_SETS_LM;
    }
    (void)AGC_ASSERT(n <= AGC_EXEC_MAX_JOBS);
    (void)AGC_ASSERT(n > 0);
    return n;
}

/* The signed priority word of core set i as CCS PRIORITY sees it:
 * positive for an active job, negative for a sleeping one, 0 (-0) free. */
static int32_t prio_word(int i)
{
    if (!AGC_ASSERT(i >= 0 && i < AGC_EXEC_MAX_JOBS)) {
        return 0;
    }
    const core_set_t *c = &core[i];
    int32_t w = ((int32_t)c->prio << EXEC_PRIO_SHIFT) + (int32_t)(c->vac + 1);
    if (!AGC_ASSERT(c->vac >= -1 && c->vac < AGC_EXEC_MAX_VACS)) {
        w = (int32_t)c->prio << EXEC_PRIO_SHIFT;
    }
    if (c->state == AGC_CORE_FREE) {
        w = 0;
    } else if (c->state == AGC_CORE_SLEEPING) {
        w = -w;
    } else {
        /* active: positive */
    }
    return w;
}

static void activity_light(bool on)
{
    /* CAF TWO / WOR DSALMOUT (NUDIRECT, NUCHANG2) and CS TWO / WAND
     * DSALMOUT (DUMMYJOB): the COMP ACTY lamp, channel 11 bit 2.        */
    agc_status_t st;
    if (on) {
        st = agc_chan_or(AGC_CHAN_DSALMOUT, (uint16_t)AGC_CH011_COMPACT);
    } else {
        st = agc_chan_and(AGC_CHAN_DSALMOUT, (uint16_t)~(uint16_t)AGC_CH011_COMPACT);
    }
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
}

/* STARTSB2 (FRESH_START_AND_RESTART, C p.191 / L p.221): make all core
 * sets and VAC areas available, NEWJOB = -0.                           */
void exec_clear_tables(void)
{
    for (int i = 0; i < AGC_EXEC_MAX_JOBS; i++) {
        core[i].state = AGC_CORE_FREE;          /* CS ZERO / TS PRIORITY */
        core[i].prio = 0;
        core[i].vac = -1;
        core[i].loc = AGC_JOB_NONE;
        core[i].arg = 0;
    }
    for (int v = 0; v < AGC_EXEC_MAX_VACS; v++) {
        vac_used[v] = false;                    /* VACnUSE = own address */
    }
    newjob = EXEC_NEWJOB_IDLE;                  /* TS NEWJOB (-0)        */
    locctr = -1;
    yielded = false;
    generation = (uint16_t)(generation + 1U);
    (void)AGC_ASSERT(core[0].state == AGC_CORE_FREE);
}

uint16_t exec_restart_generation(void)
{
    (void)AGC_ASSERT(newjob >= EXEC_NEWJOB_IDLE);
    (void)AGC_ASSERT(newjob < AGC_EXEC_MAX_JOBS);
    return generation;
}

int exec_running_job_prio(void)
{
    if (!AGC_ASSERT(newjob < AGC_EXEC_MAX_JOBS)) {
        return -1;
    }
    if (newjob == EXEC_NEWJOB_IDLE || core[0].state != AGC_CORE_ACTIVE) {
        return -1;
    }
    return (int)core[0].prio;
}

agc_status_t agc_exec_init(void)
{
    exec_clear_tables();
    activity_light(false);
    if (!AGC_ASSERT(agc_exec_core_sets() <= AGC_EXEC_MAX_JOBS)) {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* SETLOC (C p.1212 / L p.1107): compare the new priority word with that
 * of core set NEWJOB (index 0 when NEWJOB is +-0) and set NEWJOB to the
 * new core set if the new job is strictly higher (BZMF ENDFIND).       */
static void setloc_compare(int i)
{
    if (!AGC_ASSERT(i >= 0 && i < agc_exec_core_sets())) {
        return;
    }
    int idx = (newjob > 0) ? newjob : 0;
    if (!AGC_ASSERT(idx < AGC_EXEC_MAX_JOBS)) {
        return;
    }
    if (prio_word(i) - prio_word(idx) > 0) {
        newjob = i;
    }
}

/* SPECTEST (C p.1212 / L p.1107): the job went into core set 0. With
 * NEWJOB = -0 (idle) it becomes the running job (NEWJOB = +0); with a
 * change pending it is compared as in SETLOC. +0 is a CCS hole.        */
static agc_status_t spectest(void)
{
    if (newjob == EXEC_NEWJOB_IDLE) {
        newjob = 0;                 /* +0 SHOWS ACTIVE JOB ALREADY SET */
        return AGC_STATUS_OK;
    }
    if (!AGC_ASSERT(newjob > 0)) {  /* TC CCSHOLE                      */
        return agc_ccshole();
    }
    setloc_compare(0);
    return AGC_STATUS_OK;
}

/* NOVAC2 / NOVAC3 / CORFOUND / NEXTCORE (C pp.1211-1212, L pp.1106-1107):
 * find a free core set (PRIORITY = -0) scanning from core set 0 up, load
 * priority and LOC, and decide about NEWJOB. No core set: BAILOUT 1202.  */
static agc_status_t novac2(agc_prio_t prio, agc_job_id_t job, int vac)
{
    if (!AGC_ASSERT(prio >= 1 && prio <= EXEC_PRIO_MAX) ||
        !AGC_ASSERT(job != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    int n = agc_exec_core_sets();
    int found = -1;
    for (int i = 0; i < n && found < 0; i++) {
        if (core[i].state == AGC_CORE_FREE) {
            found = i;
        }
    }
    if (found < 0) {
        return agc_bailout(AGC_ALARM_01202);    /* NO CORE SETS */
    }
    core[found].state = AGC_CORE_ACTIVE;
    core[found].prio = prio;
    core[found].vac = vac;
    core[found].loc = job;
    core[found].arg = 0;
    locctr = found;
    if (found == 0) {
        return spectest();
    }
    setloc_compare(found);
    return AGC_STATUS_OK;
}

/* NOVAC (C p.1208 / L p.1103): enter a basic job that needs no VAC area.
 * prio is the priority number (PRIOnn >> 9).                           */
agc_status_t agc_exec_novac(agc_prio_t prio, agc_job_id_t job)
{
    if (!AGC_ASSERT(prio >= 1 && prio <= EXEC_PRIO_MAX) ||
        !AGC_ASSERT(job != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    agc_status_t st = novac2(prio, job, -1);
    agc_relint();
    return st;
}

/* FINDVAC / FINDVAC2 / VACFOUND (C pp.1208,1211 / L pp.1103,1106): claim
 * the first free VAC area (VAC1USE..VAC5USE in order), then NOVAC2.
 * No VAC area: BAILOUT 1201.                                           */
agc_status_t agc_exec_findvac(agc_prio_t prio, agc_job_id_t job)
{
    if (!AGC_ASSERT(prio >= 1 && prio <= EXEC_PRIO_MAX) ||
        !AGC_ASSERT(job != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    int vac = -1;
    for (int v = 0; v < AGC_EXEC_MAX_VACS && vac < 0; v++) {
        if (!vac_used[v]) {
            vac = v;
        }
    }
    agc_status_t st;
    if (vac < 0) {
        st = agc_bailout(AGC_ALARM_01201);      /* NO VAC AREAS */
    } else {
        vac_used[vac] = true;                   /* store +0 in VACnUSE */
        st = novac2(prio, job, vac);
    }
    agc_relint();
    return st;
}

/* "LOCCTR is left set at this core set if the caller wants to load any
 * MPAC registers": store the word the new job finds in its MPAC (A/L). */
agc_status_t agc_exec_set_new_job_arg(agc_sp_t arg)
{
    if (!AGC_ASSERT(locctr >= 0 && locctr < AGC_EXEC_MAX_JOBS)) {
        return AGC_STATUS_NOT_FOUND;
    }
    if (!AGC_ASSERT(core[locctr].state == AGC_CORE_ACTIVE)) {
        return AGC_STATUS_NOT_FOUND;
    }
    core[locctr].arg = arg;
    return AGC_STATUS_OK;
}

/* CHANJOB (C p.1213 / L p.1108): swap core set 0 with core set NEWJOB and
 * set NEWJOB to +0. MPAC/OVFIND/PUSHLOC swapping is not modelled.      */
static void chanjob(int k)
{
    if (!AGC_ASSERT(k > 0 && k < agc_exec_core_sets())) {
        return;
    }
    core_set_t tmp = core[0];
    core[0] = core[k];
    core[k] = tmp;
    newjob = 0;
    (void)AGC_ASSERT(core[0].state == AGC_CORE_ACTIVE);
}

/* EJSCAN / EJ1 / EJ2 (C pp.1218-1219 / L pp.1113-1115): find the core set
 * (1..n-1) with the highest active priority word strictly above `floor`
 * (BUF +1 holds -floor; -0 for ENDOFJOB / JOBSLEEP). Returns -1 if none.*/
static int ejscan(int32_t floor)
{
    if (!AGC_ASSERT(floor >= 0)) {
        return -1;
    }
    int n = agc_exec_core_sets();
    int best = -1;
    int32_t high = floor;
    for (int i = 1; i < n; i++) {
        int32_t w = prio_word(i);
        if (w > 0 && w - high > 0) {           /* EJ1: CCS of new-old  */
            high = w;
            best = i;
        }
    }
    (void)AGC_ASSERT(best < n);
    return best;
}

/* After ENDOFJOB or JOBSLEEP: change to the best waiting job or idle.  */
static void ejscan_change(void)
{
    int best = ejscan(0);
    if (best < 0) {
        newjob = EXEC_NEWJOB_IDLE;             /* DUMMYJOB: NEWJOB = -0 */
        activity_light(false);
    } else {
        chanjob(best);                          /* NEWJOB, CHANJOB -2   */
    }
    (void)AGC_ASSERT(newjob <= 0);
}

/* ENDOFJOB / ENDJOB1 (C pp.1210,1218 / L pp.1105,1113): release core set
 * 0 and, for a FINDVAC job, its VAC area; then EJSCAN.                 */
static void end_of_job(void)
{
    if (!AGC_ASSERT(core[0].state != AGC_CORE_FREE)) {
        return;
    }
    if (core[0].vac >= 0 && AGC_ASSERT(core[0].vac < AGC_EXEC_MAX_VACS)) {
        vac_used[core[0].vac] = false;          /* INDEX A / TS 0      */
    }
    core[0].state = AGC_CORE_FREE;              /* XCH PRIORITY (-0)   */
    core[0].vac = -1;
    core[0].loc = AGC_JOB_NONE;
    ejscan_change();
}

/* JOBSLEEP (C p.1210 / L p.1105): the running job goes dormant; it will
 * continue as `wake_as` when JOBWAKE names that id. The EJSCAN that the
 * AGC does at once (JOBSLP1) happens when the job body returns.        */
agc_status_t agc_exec_job_sleep(agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(wake_as != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(newjob >= 0 && core[0].state == AGC_CORE_ACTIVE)) {
        return AGC_STATUS_BAD_ARG;              /* not called from a job */
    }
    core[0].loc = wake_as;                      /* TS LOC              */
    core[0].state = AGC_CORE_SLEEPING;          /* CS PRIORITY         */
    return AGC_STATUS_OK;
}

/* JOBWAKE / JOBWAKE2-4 / WAKETEST (C pp.1210,1216 / L pp.1105,1111): find
 * the sleeping job whose LOC is `job`, make it active again and treat it
 * like a new job for NEWJOB. Not found: LOCCTR = -1, NOT_FOUND.        */
agc_status_t agc_exec_job_wake(agc_job_id_t job)
{
    if (!AGC_ASSERT(job != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    int n = agc_exec_core_sets();
    int found = -1;
    for (int i = 0; i < n && found < 0; i++) {
        if (core[i].state == AGC_CORE_SLEEPING && core[i].loc == job) {
            found = i;
        }
    }
    agc_status_t st = AGC_STATUS_OK;
    if (found < 0) {
        locctr = -1;                            /* CS ONE / TS LOCCTR  */
        st = AGC_STATUS_NOT_FOUND;
    } else {
        core[found].state = AGC_CORE_ACTIVE;    /* re-complement PRIORITY */
        locctr = found;
        if (found == 0) {
            st = spectest();
        } else {
            setloc_compare(found);
        }
    }
    agc_relint();
    return st;
}

/* PRIOCHNG / PRIOCH2 (C pp.1210,1217 / L pp.1105,1112): give the running
 * job a new priority, keeping its VAC bits. Returns OK if it is still the
 * highest (BUF = 0); BUSY if a waiting job is now strictly higher: NEWJOB
 * is set and the caller must agc_exec_job_yield() and return, which is
 * where the AGC would have done the change-job.                         */
agc_status_t agc_exec_prio_change(agc_prio_t new_prio)
{
    if (!AGC_ASSERT(new_prio >= 1 && new_prio <= EXEC_PRIO_MAX)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(newjob >= 0 && core[0].state == AGC_CORE_ACTIVE)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_inhint();
    core[0].prio = new_prio;
    int best = ejscan(prio_word(0));
    agc_status_t st = AGC_STATUS_OK;
    if (best > 0) {
        newjob = best;
        st = AGC_STATUS_BUSY;
    } else {
        newjob = 0;                 /* ENDPRCHG -1: TS NEWJOB (+0)     */
    }
    agc_relint();
    return st;
}

/* CHANG1 / CHANG2 (C p.1208 / L p.1103): the running job offers the
 * processor; it continues as `resume_as` the next time it runs.        */
agc_status_t agc_exec_job_yield(agc_job_id_t resume_as)
{
    if (!AGC_ASSERT(resume_as != AGC_JOB_NONE)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (!AGC_ASSERT(newjob >= 0 && core[0].state == AGC_CORE_ACTIVE)) {
        return AGC_STATUS_BAD_ARG;
    }
    core[0].loc = resume_as;                    /* LOC = return point  */
    yielded = true;
    return AGC_STATUS_OK;
}

/* What the job's exit did in the AGC: JOBSLP1, CHANG1 or ENDOFJOB.     */
static void after_job(void)
{
    if (core[0].state == AGC_CORE_SLEEPING) {
        ejscan_change();                        /* JOBSLP1 -> EJSCAN   */
    } else if (yielded) {
        if (newjob > 0) {
            chanjob(newjob);                    /* CHANG1 -> CHANJOB   */
        }
    } else {
        end_of_job();                           /* ENDOFJOB            */
    }
    (void)AGC_ASSERT(newjob <= 0 || core[0].state == AGC_CORE_ACTIVE);
}

/* DUMMYJOB / ADVAN / NUCHANG2 / NUDIRECT (C p.1220 / L p.1116): run the
 * highest-priority ready job once. Returns NOT_FOUND when idle (the AGC
 * would sit in DUMMYJOB / SELFCHK), otherwise the job's own status.    */
agc_status_t agc_exec_run_one(void)
{
    if (!AGC_ASSERT(newjob >= EXEC_NEWJOB_IDLE && newjob < AGC_EXEC_MAX_JOBS)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (newjob == EXEC_NEWJOB_IDLE) {
        return AGC_STATUS_NOT_FOUND;            /* DUMMYJOB            */
    }
    if (newjob > 0) {
        chanjob(newjob);                        /* NUCHANG2 / CHANJOB  */
    }
    if (!AGC_ASSERT(core[0].state == AGC_CORE_ACTIVE)) {
        return agc_ccshole();
    }
    activity_light(true);                       /* NUDIRECT            */
    agc_job_ctx_t ctx;
    ctx.id = core[0].loc;
    ctx.prio = core[0].prio;
    ctx.has_vac = core[0].vac >= 0;
    ctx.vac_index = core[0].vac;
    ctx.arg = core[0].arg;
    yielded = false;
    uint16_t gen = generation;
    in_job = true;
    agc_status_t st = agc_dispatch_job(&ctx);
    in_job = false;
    if (agc_exec_restart_pending()) {
        /* BAILOUT / POODOO / ABORT in the job: WHIMPER -> ENEMA. The
         * job's core set is wiped by STARTSB2, so no ENDOFJOB.        */
        agc_status_t rs = agc_exec_service_restart();
        (void)AGC_ASSERT(rs == AGC_STATUS_OK || rs == AGC_STATUS_BAILOUT);
    } else if (gen == generation) {
        after_job();
    } else {
        /* a fresh start (V36 SLAP1) re-initialised the tables in the job */
    }
    return st;
}

bool exec_in_job(void)
{
    (void)AGC_ASSERT(newjob >= EXEC_NEWJOB_IDLE);
    (void)AGC_ASSERT(newjob < AGC_EXEC_MAX_JOBS);
    return in_job;
}

agc_job_id_t exec_running_job_loc(void)
{
    if (!AGC_ASSERT(newjob < AGC_EXEC_MAX_JOBS)) {
        return AGC_JOB_NONE;
    }
    return (core[0].state == AGC_CORE_FREE) ? AGC_JOB_NONE : core[0].loc;
}

/* A job whose 2CADR is ENDOFJOB (LM RESTART_TABLES 1.2SPOT, p.239): it
 * ends as soon as it is dispatched.                                    */
agc_status_t agc_job_endofjob(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != 0)) {
        return AGC_STATUS_BAD_ARG;
    }
    (void)AGC_ASSERT(ctx->id == AGC_JOB_ENDOFJOB);
    return AGC_STATUS_OK;
}

bool agc_exec_idle(void)
{
    (void)AGC_ASSERT(newjob >= EXEC_NEWJOB_IDLE);
    (void)AGC_ASSERT(newjob < AGC_EXEC_MAX_JOBS);
    return newjob == EXEC_NEWJOB_IDLE;
}

int agc_exec_newjob(void)
{
    (void)AGC_ASSERT(newjob >= EXEC_NEWJOB_IDLE);
    (void)AGC_ASSERT(newjob < AGC_EXEC_MAX_JOBS);
    return newjob;
}

agc_status_t agc_exec_core_info(int core_set, agc_exec_core_info_t *out)
{
    if (!AGC_ASSERT(out != 0) ||
        !AGC_ASSERT(core_set >= 0 && core_set < AGC_EXEC_MAX_JOBS)) {
        return AGC_STATUS_BAD_ARG;
    }
    out->state = core[core_set].state;
    out->prio = core[core_set].prio;
    out->vac_index = core[core_set].vac;
    out->loc = core[core_set].loc;
    return AGC_STATUS_OK;
}
