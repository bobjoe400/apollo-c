/**********************************************************************
 * FILE NAME: uplink_update.c
 *
 * PURPOSE:
 *   P27 UPLINK UPDATE, verbs 70-73: the ground (or crew) loads the update
 *   word by word with V21 N01, verifies it with V21 N02 / V33, and P27
 *   stores it: liftoff time (V70), a block of erasable (V71), scattered
 *   words (V72) or the AGC clock (V73).
 *
 * AGC SOURCE:
 *   Comanche055 UPDATE_PROGRAM.agc pp.1491-1500 region (V70UPDAT .. UPACTOFF)
 *   Luminary099 UPDATE_PROGRAM.agc pp.1379-1388 (no CKMDMORE)
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors)
 *   Name           Type      I/O  Scale     Description
 *   -------------  --------  ---  --------  ------------------------------
 *   MODREG         agc_sp_t  I/O  unscaled  major mode (saved in UPOLDMOD)
 *   TIME2, TIME1   uint32_t  I/O  cs 2^28   the AGC clock (V70, V73)
 *   TETCSM/TETLEM  agc_dp_t  I/O  cs 2^28   state vector times (V70)
 *   TEPHEM         agc_tp_t  I/O  cs 2^42   ephemeris time (V70)
 *   any erasable   uint16_t  O    words     V71 / V72 targets (ECADR image)
 *   VERIFLAG       flag      I/O  -         inverted on V33 (FLAGWRD7 bit 3)
 *
 * EXTERNAL REFERENCES:
 *   Name                     Description
 *   -----------------------  ------------------------------------------
 *   agc_extverb_testxact     TESTXACT
 *   agc_dsky_display         GOXDSPF V21N01 / V21N02
 *   agc_integ_install/       INTSTALL, INTWAKEU
 *    intwakeu
 *   agc_phase_*              PHASCHNG group 6, NEWMODEX
 *   agc_erase_write_ecadr    the V71 / V72 stores
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Operator error light (FALTON) for a mode that may not be interrupted,
 *   a V71 block that would cross an EBANK, an even V72 count, or a clock
 *   increment that overflows; no restart group is left set after an error.
 *
 * NOTES:
 *     V70  liftoff time increment: TEPHEM +delta, clock/TETCSM/TETLEM -delta
 *     V71  contiguous block update at an ECADR (3 <= II <= 20D components)
 *     V72  scatter update, (II-1)/2 address/word pairs (II odd)
 *     V73  octal AGC clock increment (the octal equivalent of V55).
 *   Every component arrives as a V21N01 load into UPBUFF; V21N02 lets the
 *   ground correct component UPTEMP; V33 on the V21N02 stores the update
 *   (VERIFLAG is inverted so the ground sees the V33 arrive).  P27 runs only
 *   in P00 (LM) or P00/P02/fresh start (CSM, FLAGWRD5 COMPUTER bit set).
 *
 *   The only vehicle difference: Luminary has no CKMDMORE (any mode
 *   other than P00 is an error), and names BIT3 of FLAGWRD7 VERIFBIT.
 *
 *   MPAC+2: the UPBUFF/UPTEMP ECADR goes to pinball with the V21N01/N02
 *   display (agc_dsky_display_ecadr), so the ground's word lands in the
 *   erasable image at its assembled address (Comanche COMPNUMB 0300,
 *   UPTEMP 0330; Luminary COMPNUMB 1170, UPTEMP 1167). DEVIATION: the
 *   block itself is kept here (up[], uptemp); the load job takes the
 *   loaded word from the image (load_target) into it, and the program's
 *   own stores to COMPNUMB .. UPCOUNT do not reach the image.
 *
 *   DEVIATION (clock): TIMEDIDL zeroes TIME2,TIME1 and adds the
 *   incremented old time back so no count is lost; in C the clock is
 *   read and set in one step under INHINT, which has the same result.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/dsky.h"
#include "agc/io/service.h"
#include "agc/nav/integration.h"

#define WORD15         077777u            /* one AGC word */
#define LOW8           000377u            /* relative address in an EBANK */
#define ECADR_MASK     003777u            /* TS EBANK, MASK LOW8: the erasable reached */
#define BIT1           000001u            /* BIT1: odd component count */
#define BIT3           000004u            /* BIT3: uplink activity lamp (channel 11), VERIFLAG */
#define BIT8           000200u            /* BIT8: FLAGWRD5 COMPUTER (CMC) */
#define BIT9           000400u            /* BIT9: carry into the next EBANK */
#define UP_BLOCK       24                 /* COMPNUMB, UPOLDMOD, UPVERB, UPCOUNT, UPBUFF 20D */
#define UP_UPBUFF      4                  /* index of UPBUFF +0 in the block           */
#define UP_UPTEMP      24                 /* agc_uplink_update_write index of UPTEMP   */
#define UP_MAX_COMP    20                 /* II .LE. 20D                               */
#define UP_MIN_COMP    3                  /* II .GE. 3                                 */
#define UP_GROUP       6                  /* restart group 6                           */
#define UP_PHASE_EXIT  07026u             /* restart: restore the old mode and exit   */
#define UP_PHASE_STORE 04026u             /* restart: redo the store                   */
#define UP_PHASE_CLOCK 5u                 /* UPDTPHAS: restart at TIMEDIDR (6.5)       */
#define UP_DNLIST      1                  /* UPDNLIST: P27 downlist code               */
#define UP_MODE        27                 /* NEWMODEX 27                               */
#define UP_PRIO        ((agc_prio_t) 030) /* CHRPRIO                          */
#define MODREG_NEG0    (-1)               /* exec's -0 MODREG (fresh start)            */
#define RECYCLE_VERB   32                 /* V32: recycle */
#define INSTALL_AREA   0                  /* INTSTALL                                  */
#define UP_EC_CSM      00300u             /* COMPNUMB ECADR, Comanche055               */
#define UP_EC_TEMP_CSM 00330u             /* UPTEMP ECADR, Comanche055                 */
#define UP_EC_LM       01170u             /* COMPNUMB ECADR, Luminary099               */
#define UP_EC_TEMP_LM  01167u             /* UPTEMP ECADR, Luminary099                 */

/* COMPNUMB .. UPBUFF +19D, in the AGC's erasable order.                    */
enum
{
    UP_COMPNUMB = 0,
    UP_UPOLDMOD = 1,
    UP_UPVERB   = 2,
    UP_UPCOUNT  = 3
};

static uint16_t up[UP_BLOCK]; /* COMPNUMB .. UPBUFF +19D, AGC words */
static uint16_t uptemp;       /* UPTEMP: component to correct        */
static uint16_t upverbsv;     /* UPVERBSV                            */
static agc_sp_t upsvflag;     /* UPSVFLAG, B(1)PRM state vector id, unscaled */

/* Which load the V21N01/N02 on the DSKY is for.                           */
typedef enum up_stage
{
    UP_COUNT = 0,
    UP_DATA,
    UP_VERIFY
} up_stage_t;

static up_stage_t stage;       /* which load is on the DSKY */
static int        load_target; /* MPAC+2: block index or UP_UPTEMP     */

/* ---- erasables ---------------------------------------------------------- */

/**********************************************************************
 * AGC: none -- COMPNUMB .. UPBUFF +19D as the downlist reads them
 * (UPDNLIST)
 *
 * ARGUMENTS:
 *   idx  int  I   0..23: COMPNUMB, UPOLDMOD, UPVERB, UPCOUNT, UPBUFF
 *                 +0..+19D
 *
 * RETURNS: the word, 15 bits
 **********************************************************************/
uint16_t agc_uplink_update_dnword(int idx)
{
    return (up[idx]);
}

/**********************************************************************
 * AGC: none -- the target of a V21N01/N02 load or an ECADR write into
 * COMPNUMB .. UPBUFF +19D (0..23) or UPTEMP (24)
 *
 * ARGUMENTS:
 *   idx   int       I   0..23 (COMPNUMB .. UPBUFF +19D) or 24 (UPTEMP)
 *   word  uint16_t  I   the loaded word, 15 bits
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for an index or word out of range
 **********************************************************************/
agc_status_t agc_uplink_update_write(int idx, uint16_t word)
{
    if (!AGC_ASSERT(idx >= 0 && idx <= UP_UPTEMP) || !AGC_ASSERT(word <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (idx == UP_UPTEMP)
    {
        uptemp = word;
    }
    else
    {
        up[idx] = word;
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- the load MPAC +2 names (UPDATE_PROGRAM OHWELL1 / OHWELL2 /
 * UPVERIFY)
 *
 * RETURNS: the index agc_uplink_update_write takes for the V21N01/N02 on
 *          the DSKY
 **********************************************************************/
int agc_uplink_update_target(void)
{
    return (load_target);
}

/**********************************************************************
 * UPSVFLAG (UPDATE_PROGRAM.agc, INTWAKEU) -- the state vector a P27
 * update replaced
 *
 * RETURNS: UPSVFLAG: 0, +-1 earth, +-2 moon (+ CSM, - LM)
 **********************************************************************/
agc_sp_t agc_erase_upsvflag(void)
{
    return (upsvflag);
}

/**********************************************************************
 * AGC: none -- UPSVFLAG, written (the ground's V71 load before a state vector update)
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I   -2..2
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG outside -2..2
 **********************************************************************/
agc_status_t agc_erase_set_upsvflag(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= -2 && v <= 2))
    {
        return (AGC_STATUS_BAD_ARG); /* not a state vector id     */
    }

    upsvflag = v;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- Fresh start (DOFSTART): UPSVFLAG = 0.
 **********************************************************************/
void agc_uplink_update_fresh_start(void)
{
    upsvflag = 0;
    stage    = UP_COUNT;
}

/**********************************************************************
 * AGC: none -- Signed value of a 15-bit ones'-complement word.
 *
 * ARGUMENTS:
 *   w  uint16_t  I   a 15-bit ones' complement word
 *
 * RETURNS: its value
 **********************************************************************/
static int32_t ones_value(uint16_t w)
{
    return (((w & 040000u) != 0u) ? -(int32_t) (WORD15 - w) : (int32_t) w); /* negative: -(77777 - w) */
}

/**********************************************************************
 * AGC: none -- DP value (cs, 2^28) of UPBUFF +k, +k+1.
 *
 * ARGUMENTS:
 *   k  int  I   UPBUFF word offset of the major word
 *
 * RETURNS: the DP value, cs 2^28
 **********************************************************************/
static int32_t upbuff_dp(int k)
{
    return (ones_value(up[UP_UPBUFF + k]) * 040000 + ones_value(up[UP_UPBUFF + k + 1]));
}

/**********************************************************************
 * AGC: none -- Store a DP value into UPBUFF +k, +k+1 (both words take its sign).
 *
 * ARGUMENTS:
 *   k  int      I   UPBUFF word offset of the major word
 *   v  int32_t  I   DP value, cs 2^28
 **********************************************************************/
static void set_upbuff_dp(int k, int32_t v)
{
    int32_t  m  = (v < 0) ? -v : v;                /* |v|, cs */
    uint16_t hi = (uint16_t) ((m >> 14) & 037777); /* major word magnitude */
    uint16_t lo = (uint16_t) (m & 037777);         /* minor word magnitude */

    up[UP_UPBUFF + k]     = (v < 0) ? (uint16_t) (WORD15 - hi) : hi; /* both words take the sign */
    up[UP_UPBUFF + k + 1] = (v < 0) ? (uint16_t) (WORD15 - lo) : lo; /* (ones' complement) */
}

/* ---- displays ------------------------------------------------------------ */

/**********************************************************************
 * AGC: none -- the ECADR of the P27 word at index idx (COMPNUMB ..
 * UPBUFF +19D, then UPTEMP), as assembled in each rope
 *
 * ARGUMENTS:
 *   idx  int  I   0..23 or UP_UPTEMP
 *
 * RETURNS: uint16_t -- the ECADR
 **********************************************************************/
static uint16_t up_ecadr(int idx)
{
    const bool lm   = (agc_state_vehicle() == AGC_VEHICLE_LM);                   /* the rope */
    uint16_t   word = (uint16_t) ((lm ? UP_EC_LM : UP_EC_CSM) + (uint16_t) idx); /* COMPNUMB + idx */

    if (idx == UP_UPTEMP)
    {
        word = lm ? UP_EC_TEMP_LM : UP_EC_TEMP_CSM;
    }

    return (word);
}

/**********************************************************************
 * AGC: none -- GOXDSPF V21N01 (load target) or V21N02 (UPTEMP) with
 * MPAC+2 = the ECADR of the word to be loaded (OHWELL1, OHWELL2,
 * UPVERIFY), so pinball's internal machine-address load (INTMCTBS)
 * stores the ground's word there.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t up_display(agc_job_ctx_t *ctx)
{
    agc_dsky_cont_t cont = {AGC_JOB_UPLINK_UPDATE_TERMINATE,
                            AGC_JOB_UPLINK_UPDATE_PROCEED,
                            AGC_JOB_UPLINK_UPDATE_LOAD};
    uint8_t         noun = (stage == UP_VERIFY) ? 2u : 1u; /* UPVRFYNV / UPLOADNV */

    return (agc_dsky_display_ecadr(ctx, AGC_DSKY_GOMARKF, agc_dsky_vn(21, noun), &cont, up_ecadr(load_target)));
}

/**********************************************************************
 * AGC: none -- OHWELL2: request component UPCOUNT (UPBUFF +UPCOUNT-1).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t request_component(agc_job_ctx_t *ctx)
{
    stage       = UP_DATA; /* UPCOUNT 1..COMPNUMB (<= 20D) */
    load_target = UP_UPBUFF + (int) up[UP_UPCOUNT] - 1;

    return (up_display(ctx));
}

/**********************************************************************
 * AGC: none -- UPVERIFY: flash V21N02 for a correction index or V33.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the running job
 *
 * RETURNS: the display request status
 **********************************************************************/
static agc_status_t request_verify(agc_job_ctx_t *ctx)
{
    stage       = UP_VERIFY;
    load_target = UP_UPTEMP;

    return (up_display(ctx));
}

/**********************************************************************
 * AGC: none -- UPACTOFF: uplink activity light off (channel 11 bit 3).
 **********************************************************************/
static void upactoff(void)
{
    agc_chan_and((int) AGC_CHAN_DSALMOUT, (uint16_t) (WORD15 & ~BIT3));
}

/**********************************************************************
 * AGC: none -- UPERROUT +2: operator error, uplink light off, ENDEXT (no restart group
 * is touched).
 *
 * RETURNS: the ENDEXT status
 **********************************************************************/
static agc_status_t up_error_exit(void)
{
    agc_dsky_falton();
    upactoff();

    return (agc_dsky_endmark());
}

/* ---- entry --------------------------------------------------------------- */

/**********************************************************************
 * AGC: none -- CKMDMORE (CSM only): the CMC accepts P27 in P00 and P02 as well.
 *
 * RETURNS: true when P27 may run (P00 or fresh start; CMC also P02)
 **********************************************************************/
static bool update_allowed(void)
{
    agc_sp_t mm = agc_erase_modreg(); /* MODREG, unscaled: MODREG */

    if (mm == 0 || mm == MODREG_NEG0)
    {
        return (true); /* P00 or fresh start   */
    }

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (false); /* UPERROR              */
    }

    /* CS FLAGWRD5, MASK BIT8: COMPUTER flag (set: the computer is the CMC). */
    return ((agc_flagword_get(5) & BIT8) != 0u && mm == 2);
}

/**********************************************************************
 * V70UPDAT .. V73UPDAT (C p.1491): UPVERB = verb - 70.
 *
 * ARGUMENTS:
 *   ctx   agc_job_ctx_t *  I   the extended-verb job
 *   verb  int              I   70..73
 *
 * RETURNS: the TESTXACT, NEWMODEX or display status; BAD_ARG for another
 *          verb
 **********************************************************************/
agc_status_t agc_uplink_update_start(agc_job_ctx_t *ctx, int verb)
{
    if (!AGC_ASSERT(verb >= 70 && verb <= 73))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    upverbsv        = (uint16_t) (verb - 70);
    bool         ok = false;                     /* TESTXACT got the display */
    agc_status_t st = agc_extverb_testxact(&ok); /* TESTXACT / NEWMODEX status */

    if (!ok)
    {
        return (st);
    }

    if (!update_allowed())
    {
        return (up_error_exit());
    }

    agc_sp_t mm = agc_erase_modreg(); /* MODREG, unscaled: MODREG to save in UPOLDMOD */

    up[UP_UPOLDMOD] = (uint16_t) ((mm < 0) ? WORD15 : (uint16_t) mm); /* -0 kept as -0 */
    up[UP_UPVERB]   = upverbsv;
    up[UP_UPCOUNT]  = 1;

    /* UPPART2: protect group 6 (a restart restores the old mode and exits),
     * select the P27 downlist, MM 27, then ask for II or the first word.     */
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_UPLINK_UPDATE_TERMINATE, AGC_TASK_NONE, 030, false, 0};

    agc_phase_change_var(UP_PHASE_EXIT, &v);
    agc_telem_select_list(UP_DNLIST);
    st = agc_phase_newmode(UP_MODE); /* NEWMODEX 27         */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (up[UP_UPVERB] == 1u || up[UP_UPVERB] == 2u)
    { /* V71, V72: OHWELL1   */
        stage       = UP_COUNT;
        load_target = UP_UPBUFF;

        return (up_display(ctx));
    }

    up[UP_COMPNUMB] = 2; /* V70, V73            */

    return (request_component(ctx));
} /* agc_uplink_update_start */

/* ---- operator responses ------------------------------------------------- */

/**********************************************************************
 * AGC: none -- V33 on a load: COUNT/DATA ask again (OHWELL1 +2, OHWELL2 +3); on the
 * V21N02 the update is verified: UPSTORE.
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display, UPEND73 or FINDVAC status
 **********************************************************************/
agc_status_t agc_job_uplink_update_proceed(agc_job_ctx_t *ctx)
{
    if (stage != UP_VERIFY)
    {
        return (up_display(ctx));
    }

    agc_inhint(); /* UPSTORE             */
    agc_flag_invert((agc_flag_id_t) AGC_FLAG_VERIFLAG);
    agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_UPLINK_UPDATE_EXECUTE, AGC_TASK_NONE, 030, false, 0};

    agc_phase_change_var(UP_PHASE_STORE, &v);
    agc_relint();

    if (up[UP_UPVERB] == 3u)
    {
        /* UPEND73: DCA UPBUFF, DXCH UPBUFF +8D -- the octal clock increment
         * becomes the incrementer TIMEDIDR adds.                              */
        up[UP_UPBUFF + 8] = up[UP_UPBUFF];
        up[UP_UPBUFF + 9] = up[UP_UPBUFF + 1];

        return (agc_job_timedidr());
    }

    return (agc_extverb_schedule(UP_PRIO, AGC_JOB_UPLINK_UPDATE_EXECUTE, true)); /* UPFNDVAC */
}

/**********************************************************************
 * AGC: none -- Data or V32 on a load (CK4V32 and the branch after it).
 *
 * ARGUMENTS:
 *   ctx  agc_job_ctx_t *  I   the job
 *
 * RETURNS: the display status
 **********************************************************************/
agc_status_t agc_job_uplink_update_load(agc_job_ctx_t *ctx)
{
    uint8_t verb = 0; /* the response verb (MPAC) */
    uint8_t noun = 0; /* the response noun */

    agc_dsky_response(&verb, &noun);

    if (verb == RECYCLE_VERB)
    { /* V32: ask again      */
        return (up_display(ctx));
    }

    uint16_t     word = 0;                                                  /* the ground's word as pinball stored it */
    agc_status_t st   = agc_erase_read_ecadr(up_ecadr(load_target), &word); /* the word at MPAC+2 */

    if (st == AGC_STATUS_OK)
    {
        st = agc_uplink_update_write(load_target, word);
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (stage == UP_COUNT)
    {
        /* Data on the V21N01 (count): 3 <= II <= 20D, else ask again.            */
        int32_t ii = ones_value(up[UP_UPBUFF]); /* II, number of components */

        if (ii < UP_MIN_COMP || ii > UP_MAX_COMP)
        {
            return (up_display(ctx)); /* OHWELL1 +2          */
        }

        up[UP_COMPNUMB] = (uint16_t) ii;
        up[UP_UPCOUNT]++; /* INCR UPCOUNT        */

        return (request_component(ctx));
    }

    if (stage == UP_VERIFY)
    {
        /* Data on the V21N02: a legal component index -> reload that component. */
        int32_t k = ones_value(uptemp); /* component index to correct */

        if (k <= 0 || k > (int32_t) up[UP_COMPNUMB])
        {
            return (request_verify(ctx));
        }

        stage       = UP_DATA;
        load_target = UP_UPBUFF + (int) k - 1; /* OHWELL2 +2          */

        return (up_display(ctx));
    }

    if ((int32_t) up[UP_COMPNUMB] - (int32_t) up[UP_UPCOUNT] <= 0)
    { /* all received        */
        return (request_verify(ctx));
    }

    up[UP_UPCOUNT]++;

    return (request_component(ctx));
} /* agc_job_uplink_update_load */

/* ---- the update ------------------------------------------------------- */

/**********************************************************************
 * AGC: none -- UPOUT +1: restore the old mode and downlist, light off, kill group 6,
 * ENDEXT.  Also the V34 return and the group 6 restart point.
 *
 * RETURNS: the NEWMODEX or ENDEXT status
 **********************************************************************/
agc_status_t agc_job_uplink_update_terminate(void)
{
    agc_sp_t     mm = (up[UP_UPOLDMOD] == WORD15) ? (agc_sp_t) MODREG_NEG0 : (agc_sp_t) up[UP_UPOLDMOD]; /* MODREG, unscaled: UPOLDMOD (-0 kept as -0) */
    agc_status_t st = agc_phase_newmode(mm);                                                             /* NEWMODEX +3         */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    agc_telem_select_list(0);
    upactoff();
    agc_phase_kill(UP_GROUP); /* -PHASE6 = -0        */

    return (agc_dsky_endmark());
}

/**********************************************************************
 * AGC: none -- UPOUT: release INTSTALL (INTWAKEU, which also finishes a state vector
 * update named by UPSVFLAG), then UPOUT +1.
 *
 * RETURNS: the INTWAKEU status, or that of UPOUT +1
 **********************************************************************/
static agc_status_t upout(void)
{
    agc_interp_t ip; /* interpreter state */

    agc_interp_init(&ip);
    agc_status_t st = agc_integ_intwakeu(&ip, agc_integ_shared_ctx(), &upsvflag, AGC_JOB_UPLINK_UPDATE_TERMINATE); /* INTWAKEU; restart at UPOUT +1 */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    return (agc_job_uplink_update_terminate());
}

/**********************************************************************
 * AGC: none -- UPERROUT: operator error, then UPOUT.
 *
 * RETURNS: the UPOUT status
 **********************************************************************/
static agc_status_t up_error_out(void)
{
    agc_dsky_falton();

    return (upout());
}

/**********************************************************************
 * AGC: none -- TIMEDIDR: add the incrementer in UPBUFF +8D (zeroed so a restart does
 * not add it twice) to the clock.  False on DP overflow (clock kept).
 *
 * RETURNS: true when the clock was changed (false: DP overflow, clock
 *          kept)
 **********************************************************************/
static bool timedidr(void)
{
    agc_inhint();
    int32_t inc = upbuff_dp(8); /* incrementer, cs */

    set_upbuff_dp(8, 0);
    int64_t t  = (int64_t) agc_time_now_cs() + (int64_t) inc;                               /* new clock, cs */
    bool    ok = (t > -(int64_t) AGC_TIME_MODULUS_CS && t < (int64_t) AGC_TIME_MODULUS_CS); /* no DP overflow */

    if (ok)
    { /* DELTATOK, TPAGREE   */
        t = (t + (int64_t) AGC_TIME_MODULUS_CS) % (int64_t) AGC_TIME_MODULUS_CS;
        agc_time_set_cs((uint32_t) t);
    }

    agc_relint();
    agc_phase_change(UP_PHASE_STORE);

    return (ok);
}

/**********************************************************************
 * AGC: none -- TIMEDIDL: old clock into UPBUFF +18D (kept for the restart), quick
 * PHASCHNG to 6.5, then TIMEDIDR.
 *
 * RETURNS: TIMEDIDR's result
 **********************************************************************/
static bool timedidl(void)
{
    set_upbuff_dp(18, (int32_t) agc_time_now_cs() & (int32_t) 037777777);
    agc_phase_newphase(UP_GROUP, UP_PHASE_CLOCK, false);

    return (timedidr());
}

/**********************************************************************
 * AGC: none -- V70 after the clock: TETCSM, TETLEM -= delta; TEPHEM += delta.
 *
 * RETURNS: the UPOUT status
 **********************************************************************/
static agc_status_t v70_tail(void)
{
    int32_t  delta = upbuff_dp(0);       /* liftoff time increment, cs 2^28 */
    agc_dp_t tc    = agc_erase_tetcsm(); /* TETCSM, cs 2^28 */
    agc_dp_t tl    = agc_erase_tetlem(); /* TETLEM, cs 2^28 */

    agc_erase_set_tetcsm((agc_dp_t) (tc - delta));               /* DAS TETCSM          */
    agc_erase_set_tetlem((agc_dp_t) (tl - delta));               /* DAS TETLEM          */
    agc_erase_set_tephem(agc_erase_tephem() + (agc_tp_t) delta); /* TEPHEM +1 */
    set_upbuff_dp(0, 0);
    agc_phase_change(UP_PHASE_STORE);

    return (upout());
}

/**********************************************************************
 * AGC: none -- Restart 6.5 (TIMEDIDR): finish the clock change, then the rest of the
 * verb (V73 exits, V70 updates the state vector times).
 *
 * RETURNS: the UPOUT +1 / UPOUT status
 **********************************************************************/
agc_status_t agc_job_timedidr(void)
{
    bool ok = (up[UP_UPVERB] == 3u) ? timedidl() : timedidr(); /* clock changed */

    if (up[UP_UPVERB] == 3u)
    { /* UPEND73             */
        if (!ok)
        {
            agc_dsky_falton();
        }

        return (agc_job_uplink_update_terminate());
    }

    return (ok ? v70_tail() : up_error_out()); /* clock changed: V70's state vector times */
}

/**********************************************************************
 * AGC: none -- UPEND71: II-2 words from UPBUFF +2 into consecutive registers from the
 * ECADR in UPBUFF +1, which may not run into the next EBANK.
 *
 * RETURNS: the UPOUT / UPERROUT status
 **********************************************************************/
static agc_status_t upend71(void)
{
    uint16_t ecadr = (uint16_t) (up[UP_UPBUFF + 1] & ECADR_MASK); /* ECADR of the first register */
    int      n     = (int) up[UP_COMPNUMB] - 2;                   /* COMPNUMB 3..20D: 1..18 words */

    if ((((ecadr & LOW8) + (uint16_t) (n - 1)) & BIT9) != 0u)
    {
        return (up_error_out()); /* next EBANK          */
    }

    for (int i = n - 1; i >= 0; i--)
    { /* STORLP71            */
        agc_status_t st = agc_erase_write_ecadr((uint16_t) (ecadr + (uint16_t) i), up[UP_UPBUFF + 2 + i]); /* store status */

        if (st != AGC_STATUS_OK)
        {
            return (up_error_out());
        }
    }

    return (upout());
}

/**********************************************************************
 * AGC: none -- UPEND72: (II-1)/2 pairs ECADR, word from UPBUFF +1; II must be odd.
 *
 * RETURNS: the UPOUT / UPERROUT status
 **********************************************************************/
static agc_status_t upend72(void)
{
    if ((up[UP_COMPNUMB] & BIT1) == 0u)
    {
        return (up_error_out());
    }

    int pairs = ((int) up[UP_COMPNUMB] - 1) / 2; /* COMPNUMB 3..19 odd: 1..9 */

    for (int j = pairs - 1; j >= 0; j--)
    { /* LDLOOP72            */
        uint16_t     ecadr = (uint16_t) (up[UP_UPBUFF + 1 + 2 * j] & ECADR_MASK);     /* ECADR of the register */
        agc_status_t st    = agc_erase_write_ecadr(ecadr, up[UP_UPBUFF + 2 + 2 * j]); /* store status */

        if (st != AGC_STATUS_OK)
        {
            return (up_error_out());
        }
    }

    return (upout());
}

/**********************************************************************
 * AGC: none -- UPJOB / UPWAKE / UPPART3: hold INTSTALL (a state vector may change),
 * restart-protect, REINTFLG, then the verb's store.
 *
 * RETURNS: AGC_STATUS_OK (asleep in INTSTALL), the INTSTALL status, or
 *          that of the store
 **********************************************************************/
agc_status_t agc_job_uplink_update_execute(void)
{
    if (up[UP_UPVERB] == 3u)
    { /* restart in UPEND73  */
        return (agc_job_timedidr());
    }

    agc_status_t st = agc_integ_install(INSTALL_AREA, AGC_JOB_UPLINK_UPDATE_EXECUTE); /* INTSTALL status */

    if (st == AGC_STATUS_BUSY)
    {
        return (AGC_STATUS_OK); /* sleeps in INTSTALL  */
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    const agc_phase_var_t v = {AGC_RESTART_JOB, AGC_JOB_UPLINK_UPDATE_EXECUTE, AGC_TASK_NONE, 0, false, 0U}; /* D clear: PHSPRDT6 */

    agc_phase_change_var(UP_PHASE_STORE, &v); /* UPWAKE: redo the store (INTSTALL lets the rerun in, N12) */
    agc_flag_set((agc_flag_id_t) AGC_FLAG_REINTFLG);

    if (up[UP_UPVERB] == 0u)
    { /* UPEND70             */
        set_upbuff_dp(8, -upbuff_dp(0));

        return (timedidl() ? v70_tail() : up_error_out()); /* TC TIMEDIDL; error: UPERROUT */
    }

    return ((up[UP_UPVERB] == 1u) ? upend71() : upend72()); /* UPEND71 / UPEND72 */
}
