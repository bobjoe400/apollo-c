/*
 * uplink_update.c -- P27 UPLINK UPDATE (UPDATE_PROGRAM.agc, Comanche055
 * pp. 1491-1500 region / Luminary099 pp. 1379-1388), verbs 70-73:
 *   V70  liftoff time increment: TEPHEM +delta, clock/TETCSM/TETLEM -delta
 *   V71  contiguous block update at an ECADR (3 <= II <= 20D components)
 *   V72  scatter update, (II-1)/2 address/word pairs (II odd)
 *   V73  octal AGC clock increment (the octal equivalent of V55).
 * Every component arrives as a V21N01 load into UPBUFF; V21N02 lets the
 * ground correct component UPTEMP; V33 on the V21N02 stores the update
 * (VERIFLAG is inverted so the ground sees the V33 arrive).  P27 runs only
 * in P00 (LM) or P00/P02/fresh start (CSM, FLAGWRD5 COMPUTER bit set).
 *
 * The only vehicle difference: Luminary has no CKMDMORE (any mode other
 * than P00 is an error), and names BIT3 of FLAGWRD7 VERIFBIT.
 *
 * DEVIATION (MPAC+2): the AGC passes the UPBUFF/UPTEMP ECADR to pinball in
 * MPAC+2 for the V21N01/N02 load.  The display interface does not carry
 * MPAC+2 yet (docs/modules/io-pinball.md), so the index of the word being
 * loaded is kept here (load_target) and the loaded word must reach
 * agc_uplink_update_write(), the routine the ECADR write of COMPNUMB ..
 * UPTEMP is to be mapped onto (requested from state-flags / io-pinball).
 *
 * DEVIATION (clock): TIMEDIDL zeroes TIME2,TIME1 and adds the incremented
 * old time back so no count is lost; in C the clock is read and set in one
 * step under INHINT, which has the same result.
 */
#include <stddef.h>
#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/exec/exec.h"
#include "agc/exec/pending.h"
#include "agc/io/dsky.h"
#include "agc/nav/integration.h"
#include "agc/io/service_internal.h"
#include "pending.h"

#define WORD15        077777u
#define LOW8          000377u
#define BIT1          000001u
#define BIT3          000004u
#define BIT8          000200u
#define BIT9          000400u
#define UP_BLOCK      24        /* COMPNUMB, UPOLDMOD, UPVERB, UPCOUNT, UPBUFF 20D */
#define UP_UPBUFF     4         /* index of UPBUFF +0 in the block           */
#define UP_UPTEMP     24        /* agc_uplink_update_write index of UPTEMP   */
#define UP_MAX_COMP   20        /* II .LE. 20D                               */
#define UP_MIN_COMP   3         /* II .GE. 3                                 */
#define UP_GROUP      6         /* restart group 6                           */
#define UP_PHASE_EXIT 07026u    /* restart: restore the old mode and exit   */
#define UP_PHASE_STORE 04026u   /* restart: redo the store                   */
#define UP_PHASE_CLOCK 5u       /* UPDTPHAS: restart at TIMEDIDR (6.5)       */
#define UP_DNLIST     1         /* UPDNLIST: P27 downlist code               */
#define UP_MODE       27        /* NEWMODEX 27                               */
#define UP_PRIO       ((agc_prio_t)030)  /* CHRPRIO                          */
#define MODREG_NEG0   (-1)      /* exec's -0 MODREG (fresh start)            */
#define RECYCLE_VERB  32
#define INSTALL_AREA  0         /* INTSTALL                                  */

/* COMPNUMB .. UPBUFF +19D, in the AGC's erasable order.                    */
enum { UP_COMPNUMB = 0, UP_UPOLDMOD = 1, UP_UPVERB = 2, UP_UPCOUNT = 3 };
static uint16_t up[UP_BLOCK];
static uint16_t uptemp;               /* UPTEMP: component to correct        */
static uint16_t upverbsv;             /* UPVERBSV                            */
static agc_sp_t upsvflag;             /* UPSVFLAG, B(1)PRM (state vector id) */

/* Which load the V21N01/N02 on the DSKY is for.                           */
typedef enum up_stage { UP_COUNT = 0, UP_DATA, UP_VERIFY } up_stage_t;
static up_stage_t stage;
static int        load_target;        /* MPAC+2: block index or UP_UPTEMP     */

/* ---- erasables ---------------------------------------------------------- */

uint16_t agc_uplink_update_dnword(int idx)
{
    if (!AGC_ASSERT(idx >= 0 && idx < UP_BLOCK)) {
        return 0;
    }
    return up[idx];
}

/* Target of an ECADR load/write into COMPNUMB..UPBUFF +19D (0..23) or
 * UPTEMP (24).                                                            */
agc_status_t agc_uplink_update_write(int idx, uint16_t word)
{
    if (!AGC_ASSERT(idx >= 0 && idx <= UP_UPTEMP) || !AGC_ASSERT(word <= WORD15)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (idx == UP_UPTEMP) {
        uptemp = word;
    } else {
        up[idx] = word;
    }
    return AGC_STATUS_OK;
}

int agc_uplink_update_target(void)
{
    (void)AGC_ASSERT(load_target >= UP_UPBUFF && load_target <= UP_UPTEMP);
    return load_target;
}

agc_sp_t agc_erase_upsvflag(void)
{
    (void)AGC_ASSERT(upsvflag >= -2 && upsvflag <= 2);
    return upsvflag;
}

void agc_erase_set_upsvflag(agc_sp_t v)
{
    if (AGC_ASSERT(v >= -2 && v <= 2)) {
        upsvflag = v;
    }
}

/* Fresh start (DOFSTART): UPSVFLAG = 0.                                   */
agc_status_t agc_uplink_update_fresh_start(void)
{
    upsvflag = 0;
    stage = UP_COUNT;
    (void)AGC_ASSERT(upsvflag == 0);
    return AGC_STATUS_OK;
}

/* Signed value of a 15-bit ones'-complement word.                        */
static int32_t ones_value(uint16_t w)
{
    (void)AGC_ASSERT(w <= WORD15);
    return ((w & 040000u) != 0u) ? -(int32_t)(WORD15 - w) : (int32_t)w;
}

/* DP value (cs, 2^28) of UPBUFF +k, +k+1.                                 */
static int32_t upbuff_dp(int k)
{
    (void)AGC_ASSERT(k >= 0 && k + 1 < UP_MAX_COMP);
    return ones_value(up[UP_UPBUFF + k]) * 040000 + ones_value(up[UP_UPBUFF + k + 1]);
}

/* Store a DP value into UPBUFF +k, +k+1 (both words take its sign).      */
static void set_upbuff_dp(int k, int32_t v)
{
    (void)AGC_ASSERT(k >= 0 && k + 1 < UP_MAX_COMP);
    int32_t m = (v < 0) ? -v : v;
    uint16_t hi = (uint16_t)((m >> 14) & 037777);
    uint16_t lo = (uint16_t)(m & 037777);
    up[UP_UPBUFF + k]     = (v < 0) ? (uint16_t)(WORD15 - hi) : hi;
    up[UP_UPBUFF + k + 1] = (v < 0) ? (uint16_t)(WORD15 - lo) : lo;
}

/* ---- displays ------------------------------------------------------------ */

/* GOXDSPF V21N01 (load target) or V21N02 (UPTEMP).                        */
static agc_status_t up_display(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dsky_cont_t cont = { AGC_JOB_UPLINK_UPDATE_TERMINATE,
                             AGC_JOB_UPLINK_UPDATE_PROCEED, AGC_JOB_UPLINK_UPDATE_LOAD };
    uint8_t noun = (stage == UP_VERIFY) ? 2u : 1u;   /* UPVRFYNV / UPLOADNV */
    return agc_dsky_display(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(21, noun), &cont);
}

/* OHWELL2: request component UPCOUNT (UPBUFF +UPCOUNT-1).                 */
static agc_status_t request_component(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(up[UP_UPCOUNT] >= 1u && up[UP_UPCOUNT] <= UP_MAX_COMP);
    stage = UP_DATA;
    load_target = UP_UPBUFF + (int)up[UP_UPCOUNT] - 1;
    return up_display(ctx);
}

/* UPVERIFY: flash V21N02 for a correction index or V33.                  */
static agc_status_t request_verify(agc_job_ctx_t *ctx)
{
    stage = UP_VERIFY;
    load_target = UP_UPTEMP;
    (void)AGC_ASSERT(up[UP_COMPNUMB] >= 2u);
    return up_display(ctx);
}

/* UPACTOFF: uplink activity light off (channel 11 bit 3).                 */
static void upactoff(void)
{
    agc_status_t st = agc_chan_and((int)AGC_CHAN_DSALMOUT, (uint16_t)(WORD15 & ~BIT3));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
}

/* UPERROUT +2: operator error, uplink light off, ENDEXT (no restart group
 * is touched).                                                            */
static agc_status_t up_error_exit(void)
{
    agc_status_t st = agc_dsky_falton();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    upactoff();
    return agc_dsky_endext();
}

/* ---- entry --------------------------------------------------------------- */

/* CKMDMORE (CSM only): the CMC accepts P27 in P00 and P02 as well.       */
static bool update_allowed(void)
{
    agc_sp_t mm = agc_erase_modreg();
    if (mm == 0 || mm == MODREG_NEG0) {
        return true;                              /* P00 or fresh start   */
    }
    if (agc_state_vehicle() == AGC_VEHICLE_LM) {
        return false;                             /* UPERROR              */
    }
    (void)AGC_ASSERT(mm > 0);
    /* CS FLAGWRD5, MASK BIT8: COMPUTER flag (set: the computer is the CMC). */
    return (agc_flagword_get(5) & BIT8) != 0u && mm == 2;
}

/* UPPART2: protect group 6 (a restart restores the old mode and exits),
 * select the P27 downlist, MM 27, then ask for II or the first word.     */
static agc_status_t uppart2(agc_job_ctx_t *ctx)
{
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_UPLINK_UPDATE_TERMINATE,
                          AGC_TASK_NONE, 030, false, 0 };
    agc_status_t st = agc_phase_change_var(UP_PHASE_EXIT, &v);
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    st = agc_telem_select_list(UP_DNLIST);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_phase_newmode(UP_MODE);                 /* NEWMODEX 27         */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if (up[UP_UPVERB] == 1u || up[UP_UPVERB] == 2u) {              /* V71, V72: OHWELL1   */
        stage = UP_COUNT;
        load_target = UP_UPBUFF;
        return up_display(ctx);
    }
    up[UP_COMPNUMB] = 2;                                    /* V70, V73            */
    return request_component(ctx);
}

/* V70UPDAT .. V73UPDAT (C p.1491): UPVERB = verb - 70.                    */
agc_status_t agc_uplink_update_start(agc_job_ctx_t *ctx, int verb)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(verb >= 70 && verb <= 73)) {
        return AGC_STATUS_BAD_ARG;
    }
    upverbsv = (uint16_t)(verb - 70);
    bool ok = false;
    agc_status_t st = agc_extverb_testxact(&ok);
    if (!ok) {
        return st;
    }
    if (!update_allowed()) {
        return up_error_exit();
    }
    agc_sp_t mm = agc_erase_modreg();
    up[UP_UPOLDMOD] = (uint16_t)((mm < 0) ? WORD15 : (uint16_t)mm);   /* -0 kept as -0 */
    up[UP_UPVERB] = upverbsv;
    up[UP_UPCOUNT] = 1;
    return uppart2(ctx);
}

/* ---- operator responses ------------------------------------------------- */

/* V33 on a load: COUNT/DATA ask again (OHWELL1 +2, OHWELL2 +3); on the
 * V21N02 the update is verified: UPSTORE.                                 */
agc_status_t agc_job_uplink_update_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (stage != UP_VERIFY) {
        return up_display(ctx);
    }
    agc_inhint();                                    /* UPSTORE             */
    agc_status_t st = agc_flag_invert((agc_flag_id_t)AGC_FLAG_VERIFLAG);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    agc_phase_var_t v = { AGC_RESTART_JOB, AGC_JOB_UPLINK_UPDATE_EXECUTE,
                          AGC_TASK_NONE, 030, false, 0 };
    st = agc_phase_change_var(UP_PHASE_STORE, &v);
    agc_relint();
    if (!AGC_ASSERT(st == AGC_STATUS_OK)) {
        return st;
    }
    if (up[UP_UPVERB] == 3u) {                              /* UPEND73             */
        return agc_job_timedidr(ctx);
    }
    return agc_extverb_schedule(UP_PRIO, AGC_JOB_UPLINK_UPDATE_EXECUTE, true); /* UPFNDVAC */
}

/* Data on the V21N01 (count): 3 <= II <= 20D, else ask again.            */
static agc_status_t got_count(agc_job_ctx_t *ctx)
{
    int32_t ii = ones_value(up[UP_UPBUFF]);
    if (ii < UP_MIN_COMP || ii > UP_MAX_COMP) {
        return up_display(ctx);                      /* OHWELL1 +2          */
    }
    up[UP_COMPNUMB] = (uint16_t)ii;
    up[UP_UPCOUNT]++;                                       /* INCR UPCOUNT        */
    return request_component(ctx);
}

/* Data on the V21N02: a legal component index -> reload that component. */
static agc_status_t got_index(agc_job_ctx_t *ctx)
{
    int32_t k = ones_value(uptemp);
    if (k <= 0 || k > (int32_t)up[UP_COMPNUMB]) {
        return request_verify(ctx);
    }
    stage = UP_DATA;
    load_target = UP_UPBUFF + (int)k - 1;            /* OHWELL2 +2          */
    return up_display(ctx);
}

/* Data or V32 on a load (CK4V32 and the branch after it).                */
agc_status_t agc_job_uplink_update_load(agc_job_ctx_t *ctx)
{
    uint8_t verb = 0;
    uint8_t noun = 0;
    agc_status_t st = agc_dsky_response(&verb, &noun);
    if (!AGC_ASSERT(ctx != NULL) || st != AGC_STATUS_OK) {
        return up_error_exit();
    }
    if (verb == RECYCLE_VERB) {                      /* V32: ask again      */
        return up_display(ctx);
    }
    if (stage == UP_COUNT) {
        return got_count(ctx);
    }
    if (stage == UP_VERIFY) {
        return got_index(ctx);
    }
    if ((int32_t)up[UP_COMPNUMB] - (int32_t)up[UP_UPCOUNT] <= 0) { /* all received        */
        return request_verify(ctx);
    }
    up[UP_UPCOUNT]++;
    return request_component(ctx);
}

/* ---- the update ------------------------------------------------------- */

/* UPOUT +1: restore the old mode and downlist, light off, kill group 6,
 * ENDEXT.  Also the V34 return and the group 6 restart point.             */
agc_status_t agc_job_uplink_update_terminate(agc_job_ctx_t *ctx)
{
    (void)AGC_ASSERT(ctx != NULL);
    agc_sp_t mm = (up[UP_UPOLDMOD] == WORD15) ? (agc_sp_t)MODREG_NEG0 : (agc_sp_t)up[UP_UPOLDMOD];
    agc_status_t st = agc_phase_newmode(mm);         /* NEWMODEX +3         */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_telem_select_list(0);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    upactoff();
    st = agc_phase_kill(UP_GROUP);                   /* -PHASE6 = -0        */
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_dsky_endext();
}

/* UPOUT: release INTSTALL (INTWAKEU, which also finishes a state vector
 * update named by UPSVFLAG), then UPOUT +1.                               */
static agc_status_t upout(agc_job_ctx_t *ctx)
{
    agc_interp_t ip;
    agc_interp_init(&ip);
    agc_status_t st = agc_integ_intwakeu(&ip, agc_integ_shared_ctx(), &upsvflag);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return agc_job_uplink_update_terminate(ctx);
}

/* UPERROUT: operator error, then UPOUT.                                    */
static agc_status_t up_error_out(agc_job_ctx_t *ctx)
{
    agc_status_t st = agc_dsky_falton();
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return upout(ctx);
}

/* TIMEDIDR: add the incrementer in UPBUFF +8D (zeroed so a restart does
 * not add it twice) to the clock.  False on DP overflow (clock kept).     */
static bool timedidr(void)
{
    agc_inhint();
    int32_t inc = upbuff_dp(8);
    set_upbuff_dp(8, 0);
    int64_t t = (int64_t)agc_time_now_cs() + (int64_t)inc;
    bool ok = (t > -(int64_t)AGC_TIME_MODULUS_CS && t < (int64_t)AGC_TIME_MODULUS_CS);
    if (ok) {                                        /* DELTATOK, TPAGREE   */
        t = (t + (int64_t)AGC_TIME_MODULUS_CS) % (int64_t)AGC_TIME_MODULUS_CS;
        agc_status_t st = agc_time_set_cs((uint32_t)t);
        (void)AGC_ASSERT(st == AGC_STATUS_OK);
    }
    agc_relint();
    agc_status_t sp = agc_phase_change(UP_GROUP, UP_PHASE_STORE);
    (void)AGC_ASSERT(sp == AGC_STATUS_OK);
    return ok;
}

/* TIMEDIDL: old clock into UPBUFF +18D (kept for the restart), quick
 * PHASCHNG to 6.5, then TIMEDIDR.                                          */
static bool timedidl(void)
{
    set_upbuff_dp(18, (int32_t)agc_time_now_cs() & (int32_t)037777777);
    agc_status_t st = agc_phase_newphase(UP_GROUP, UP_PHASE_CLOCK, false);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return timedidr();
}

/* V70 after the clock: TETCSM, TETLEM -= delta; TEPHEM += delta.         */
static agc_status_t v70_tail(agc_job_ctx_t *ctx)
{
    int32_t delta = upbuff_dp(0);
    agc_dp_t tc = agc_erase_tetcsm();
    agc_dp_t tl = agc_erase_tetlem();
    (void)AGC_ASSERT(delta > -(int32_t)AGC_TIME_MODULUS_CS);
    agc_erase_set_tetcsm((agc_dp_t)(tc - delta));    /* DAS TETCSM          */
    agc_erase_set_tetlem((agc_dp_t)(tl - delta));    /* DAS TETLEM          */
    agc_erase_set_tephem(agc_erase_tephem() + (agc_tp_t)delta);  /* TEPHEM +1 */
    set_upbuff_dp(0, 0);
    agc_status_t st = agc_phase_change(UP_GROUP, UP_PHASE_STORE);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    return upout(ctx);
}

/* Restart 6.5 (TIMEDIDR): finish the clock change, then the rest of the
 * verb (V73 exits, V70 updates the state vector times).                  */
agc_status_t agc_job_timedidr(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(up[UP_UPVERB] == 0u || up[UP_UPVERB] == 3u)) {
        return AGC_STATUS_BAD_ARG;
    }
    bool ok = (up[UP_UPVERB] == 3u) ? timedidl() : timedidr();
    if (up[UP_UPVERB] == 3u) {                              /* UPEND73             */
        if (!ok) {
            agc_status_t st = agc_dsky_falton();
            (void)AGC_ASSERT(st == AGC_STATUS_OK);
        }
        return agc_job_uplink_update_terminate(ctx);
    }
    return ok ? v70_tail(ctx) : up_error_out(ctx);
}

/* UPEND71: II-2 words from UPBUFF +2 into consecutive registers from the
 * ECADR in UPBUFF +1, which may not run into the next EBANK.              */
static agc_status_t upend71(agc_job_ctx_t *ctx)
{
    uint16_t ecadr = up[UP_UPBUFF + 1];
    int n = (int)up[UP_COMPNUMB] - 2;
    if (!AGC_ASSERT(n >= 1 && n <= UP_MAX_COMP - 2)) {
        return up_error_out(ctx);
    }
    if ((((ecadr & LOW8) + (uint16_t)(n - 1)) & BIT9) != 0u) {
        return up_error_out(ctx);                    /* next EBANK          */
    }
    for (int i = n - 1; i >= 0; i--) {               /* STORLP71            */
        agc_status_t st = agc_erase_write_ecadr((uint16_t)(ecadr + (uint16_t)i),
                                                up[UP_UPBUFF + 2 + i]);
        if (st != AGC_STATUS_OK) {
            return up_error_out(ctx);
        }
    }
    return upout(ctx);
}

/* UPEND72: (II-1)/2 pairs ECADR, word from UPBUFF +1; II must be odd.     */
static agc_status_t upend72(agc_job_ctx_t *ctx)
{
    if ((up[UP_COMPNUMB] & BIT1) == 0u) {
        return up_error_out(ctx);
    }
    int pairs = ((int)up[UP_COMPNUMB] - 1) / 2;
    (void)AGC_ASSERT(pairs >= 1 && pairs <= UP_MAX_COMP / 2);
    for (int j = pairs - 1; j >= 0; j--) {           /* LDLOOP72            */
        agc_status_t st = agc_erase_write_ecadr(up[UP_UPBUFF + 1 + 2 * j],
                                                up[UP_UPBUFF + 2 + 2 * j]);
        if (st != AGC_STATUS_OK) {
            return up_error_out(ctx);
        }
    }
    return upout(ctx);
}

/* UPJOB / UPWAKE / UPPART3: hold INTSTALL (a state vector may change),
 * restart-protect, REINTFLG, then the verb's store.                      */
agc_status_t agc_job_uplink_update_execute(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) || !AGC_ASSERT(up[UP_UPVERB] <= 3u)) {
        return AGC_STATUS_BAD_ARG;
    }
    if (up[UP_UPVERB] == 3u) {                              /* restart in UPEND73  */
        return agc_job_timedidr(ctx);
    }
    agc_status_t st = agc_integ_install(INSTALL_AREA, AGC_JOB_UPLINK_UPDATE_EXECUTE);
    if (st == AGC_STATUS_BUSY) {
        return AGC_STATUS_OK;                        /* sleeps in INTSTALL  */
    }
    st = agc_phase_change(UP_GROUP, UP_PHASE_STORE);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    st = agc_flag_set((agc_flag_id_t)AGC_FLAG_REINTFLG);
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    if (up[UP_UPVERB] == 0u) {                              /* UPEND70             */
        set_upbuff_dp(8, -upbuff_dp(0));
        return timedidl() ? v70_tail(ctx) : up_error_out(ctx);
    }
    return (up[UP_UPVERB] == 1u) ? upend71(ctx) : upend72(ctx);
}
