/**********************************************************************
 * FILE NAME: state.c
 *
 * PURPOSE:
 *   Vehicle selection, the TIME2/TIME1 centisecond clock, the flagwords
 *   (FLAGWRD0..FLGWRD13 with the interpreter switch operations SET,
 *   CLEAR, INVERT, BON/BOFF) and the bundled state-vector accessors.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc p. 39 (TIME2, TIME1), pp. 46-68
 *   (FLAGWRD0..FLGWRD11, STATE); INTERPRETER.agc p. 1129 (SWBRANCH),
 *   p. 1197 (SWITCHES)
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc p. 92 (TIME2, TIME1);
 *   FLAGWORD_ASSIGNMENTS.agc pp. 61-88 (FLAGWRD0..FLGWRD13);
 *   INTERPRETER.agc p. 1024 (SWBRANCH), p. 1092 (SWITCHES)
 *
 * EXTERNAL VARIABLES:  (module-static)
 *   Name          Type      I/O  Scale          Description
 *   ------------  --------  ---  -------------  ----------------------
 *   g_vehicle     enum      I/O  --             CSM or LM program
 *   g_time_cs     uint32_t  I/O  cs, 2^28       C(TIME2,TIME1)
 *   g_flagword    uint16_t  I/O  bit words      STATE +0..+13D
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_erase_reset       zero the shared erasables (erasable.c)
 *   agc_chan_reset        power-up channel image (channels.c)
 *   agc_erase_rn ...      state-vector erasables (erasable.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No AGC alarms. A failed AGC_ASSERT (bad vehicle, flag id not valid
 *   for the vehicle, time >= 2^28) leaves the state unchanged; getters
 *   return 0/false, agc_sv_get/agc_sv_set return AGC_STATUS_BAD_ARG.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The flagwords are an array indexed by n, not consecutive erasable
 *   addresses, so STATE+n address arithmetic is replaced by flag ids.
 *   CSM programs have 12 flagwords, LM programs 14 (RADMODES, DAPBOOLS).
 *
 * NOTES:
 *   DEVIATION: agc_sv_set also sets or clears the moon flag of the
 *   vector (MOONFLAG for THIS, LMOONFLG/CMOONFLG for OTHER); in the AGC
 *   the integration code does that. docs/modules/state-flags.md.
 *
 * ALGORITHM:
 *   Flag id = (flagword << 4) | bit, bit = AGC bit 1..15; mask =
 *   1 << (bit - 1). TIME1 is the low 14 bits of the 28-bit count,
 *   TIME2 the high 14; TIME2 overflow wraps the count to zero.
 **********************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"

#define AGC_WORD15_MASK 0x7FFFu /* a flagword holds 15 bits             */
#define AGC_TIME1_BITS  14      /* TIME1 holds the low 14 bits          */
#define AGC_TIME_FIELD  0x3FFFu /* one 14-bit TIME1 or TIME2 field      */
#define FLAG_BIT_FIELD  0xFu    /* bit number field of a flag id        */
#define FLAG_ID_MAX     0xFFu   /* largest encodable flag id            */
#define FLAG_BIT_MAX    15      /* AGC bit numbers run 1..15            */

static agc_vehicle_t g_vehicle = AGC_VEHICLE_CSM; /* selected program */
static uint32_t      g_time_cs;                   /* C(TIME2,TIME1), cs, 28 bits        */
static uint16_t      g_flagword[AGC_FLAGWORDS];   /* FLAGWRD0..FLGWRD13 */

/**********************************************************************
 * AGC: none (power-up: select the rope and zero erasable memory; the
 *      fresh-start settings are FRESH_START_AND_RESTART, exec-sched)
 *
 * FUNCTION NAME: agc_state_init
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   v         agc_vehicle_t  I    AGC_VEHICLE_CSM (Comanche055) or
 *                                 AGC_VEHICLE_LM (Luminary099)
 **********************************************************************/
void agc_state_init(agc_vehicle_t v)
{
    g_vehicle = v;
    g_time_cs = 0;

    for (int i = 0; i < AGC_FLAGWORDS; i++)
    {
        g_flagword[i] = 0;
    }

    agc_erase_reset();
    agc_chan_reset();
}

/**********************************************************************
 * AGC: none (which rope this build runs)
 *
 * FUNCTION NAME: agc_state_vehicle
 *
 * RETURNS: agc_vehicle_t -- the vehicle given to agc_state_init (CSM
 *          after an assertion report)
 **********************************************************************/
agc_vehicle_t agc_state_vehicle(void)
{
    return (g_vehicle);
}

/**********************************************************************
 * AGC: TIME2, TIME1 (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read
 *      the clock as one 28-bit count
 *
 * FUNCTION NAME: agc_time_now_cs
 *
 * RETURNS: uint32_t -- C(TIME2,TIME1), cs, 0..2^28-1 (0 after an
 *          assertion report)
 **********************************************************************/
uint32_t agc_time_now_cs(void)
{
    return (g_time_cs);
}

/**********************************************************************
 * AGC: TIME2, TIME1 (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- load
 *      the clock (P11 zeroes it at liftoff; V55 increments it)
 *
 * FUNCTION NAME: agc_time_set_cs
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   t_cs      uint32_t  I    new C(TIME2,TIME1), cs, < 2^28 (else
 *                            ignored after an assertion report)
 **********************************************************************/
void agc_time_set_cs(uint32_t t_cs)
{
    g_time_cs = t_cs;
}

/**********************************************************************
 * AGC: TIME1 counter (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) --
 *      advance the clock; TIME1 overflow increments TIME2, TIME2
 *      overflow wraps to zero
 *
 * FUNCTION NAME: agc_time_advance_cs
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   dt_cs     uint16_t  I    elapsed time, cs
 **********************************************************************/
void agc_time_advance_cs(uint16_t dt_cs)
{
    g_time_cs = (g_time_cs + dt_cs) & (AGC_TIME_MODULUS_CS - 1u);
}

/**********************************************************************
 * AGC: TIME1 (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92, register 025)
 *
 * FUNCTION NAME: agc_erase_time1
 *
 * RETURNS: agc_sp_t -- low 14 bits of the count, cs (0 after an
 *          assertion report)
 **********************************************************************/
agc_sp_t agc_erase_time1(void)
{
    return ((agc_sp_t) (g_time_cs & AGC_TIME_FIELD));
}

/**********************************************************************
 * AGC: TIME2 (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92, register 024)
 *
 * FUNCTION NAME: agc_erase_time2
 *
 * RETURNS: agc_sp_t -- high 14 bits of the count, units of 2^14 cs (0
 *          after an assertion report)
 **********************************************************************/
agc_sp_t agc_erase_time2(void)
{
    return ((agc_sp_t) ((g_time_cs >> AGC_TIME1_BITS) & AGC_TIME_FIELD));
}

/**********************************************************************
 * AGC: none (helper) -- number of flagwords in this vehicle's rope
 *
 * FUNCTION NAME: words_for_vehicle
 *
 * RETURNS: int -- 12 (Comanche055 FLAGWRD0..FLGWRD11) or 14
 *          (Luminary099 FLAGWRD0..FLGWRD13)
 **********************************************************************/
static int words_for_vehicle(void)
{
    int words = AGC_FLAGWORDS_CSM; /* CSM unless the LM is selected */

    if (g_vehicle == AGC_VEHICLE_LM)
    {
        words = AGC_FLAGWORDS_LM;
    }

    return (words);
}

/**********************************************************************
 * AGC: none (flag id check; ERASABLE_ASSIGNMENTS C pp. 46-68, FLAGWORD_
 *      ASSIGNMENTS L pp. 61-88 define the words and bits)
 *
 * FUNCTION NAME: agc_flag_valid
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id (flagword << 4) | bit
 *
 * RETURNS: bool -- true when the flagword exists in this vehicle and
 *          the bit is 1..15
 **********************************************************************/
bool agc_flag_valid(agc_flag_id_t f)
{
    const int word = (int) (f >> 4);             /* flagword index n */
    const int bit  = (int) (f & FLAG_BIT_FIELD); /* AGC bit 1..15    */

    return ((word < words_for_vehicle()) && (bit >= 1) && (bit <= FLAG_BIT_MAX));
}

/**********************************************************************
 * AGC: none (flagword index n of FLAGWRDn = STATE +n)
 *
 * FUNCTION NAME: agc_flag_word
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id
 *
 * RETURNS: int -- flagword index, or -1 for an invalid id
 **********************************************************************/
int agc_flag_word(agc_flag_id_t f)
{
    return ((int) (f >> 4));
}

/**********************************************************************
 * AGC: none (AGC bit number of the flag, as printed in the listing)
 *
 * FUNCTION NAME: agc_flag_bit
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id
 *
 * RETURNS: int -- bit number 1..15, or -1 for an invalid id
 **********************************************************************/
int agc_flag_bit(agc_flag_id_t f)
{
    return ((int) (f & FLAG_BIT_FIELD));
}

/**********************************************************************
 * AGC: SWITCHES (INTERPRETER.agc C p. 1197, L p. 1092: the bit of the
 *      switch word, SWBIT)
 *
 * FUNCTION NAME: agc_flag_mask
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id
 *
 * RETURNS: uint16_t -- 1 << (bit - 1), or 0 for an invalid id
 **********************************************************************/
uint16_t agc_flag_mask(agc_flag_id_t f)
{
    return ((uint16_t) (1u << ((f & FLAG_BIT_FIELD) - 1u)));
}

/**********************************************************************
 * AGC: BON / BOFF (INTERPRETER.agc C p. 1129, L p. 1024, SWBRANCH) --
 *      test a flag
 *
 * FUNCTION NAME: agc_flag_test
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id
 *
 * RETURNS: bool -- true when the flag is set (false for an invalid id)
 **********************************************************************/
bool agc_flag_test(agc_flag_id_t f)
{
    return ((g_flagword[f >> 4] & agc_flag_mask(f)) != 0u);
}

/**********************************************************************
 * AGC: SET (INTERPRETER.agc C p. 1197, L p. 1092, SWITCHES)
 *
 * FUNCTION NAME: agc_flag_set
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id to set
 **********************************************************************/
void agc_flag_set(agc_flag_id_t f)
{
    g_flagword[f >> 4] = (uint16_t) (g_flagword[f >> 4] | agc_flag_mask(f));
}

/**********************************************************************
 * AGC: CLEAR (INTERPRETER.agc C p. 1197, L p. 1092, SWITCHES)
 *
 * FUNCTION NAME: agc_flag_clear
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id to clear
 **********************************************************************/
void agc_flag_clear(agc_flag_id_t f)
{
    g_flagword[f >> 4] = (uint16_t) (g_flagword[f >> 4] & ~agc_flag_mask(f));
}

/**********************************************************************
 * AGC: INVERT (INTERPRETER.agc C p. 1197, L p. 1092, SWITCHES)
 *
 * FUNCTION NAME: agc_flag_invert
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id to complement
 **********************************************************************/
void agc_flag_invert(agc_flag_id_t f)
{
    g_flagword[f >> 4] = (uint16_t) (g_flagword[f >> 4] ^ agc_flag_mask(f));
}

/**********************************************************************
 * AGC: SET or CLEAR (INTERPRETER.agc C p. 1197, L p. 1092) chosen by a
 *      computed condition: the AGC tests and branches to one of the
 *      two instructions; here the condition is the argument
 *
 * FUNCTION NAME: agc_flag_put
 *
 * ARGUMENTS:
 *   Argument  Type           I/O  Description (scale)
 *   --------  -------------  ---  ---------------------------------
 *   f         agc_flag_id_t  I    flag id
 *   on        bool           I    true = SET, false = CLEAR
 **********************************************************************/
void agc_flag_put(agc_flag_id_t f, bool on)
{
    if (on)
    {
        agc_flag_set(f);
    }
    else
    {
        agc_flag_clear(f);
    }
}

/**********************************************************************
 * AGC: FLAGWRDn (STATE +n; ERASABLE_ASSIGNMENTS C pp. 46-68,
 *      FLAGWORD_ASSIGNMENTS L pp. 61-88) -- read a whole flagword
 *
 * FUNCTION NAME: agc_flagword_get
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   word      int   I    flagword index n, 0..11 (CSM) / 0..13 (LM)
 *
 * RETURNS: uint16_t -- the 15-bit flagword (0 for a bad index)
 **********************************************************************/
uint16_t agc_flagword_get(int word)
{
    return (g_flagword[word]);
}

/**********************************************************************
 * AGC: FLAGWRDn (STATE +n; ERASABLE_ASSIGNMENTS.agc C pp. 46-68,
 *      FLAGWORD_ASSIGNMENTS.agc L pp. 61-88) -- store a whole flagword
 *      (fresh start, RADMODES and DAPBOOLS whole-word updates, uplink)
 *
 * FUNCTION NAME: agc_flagword_set
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ---------------------------------------
 *   word      int       I    flagword index n, 0..11 (CSM) / 0..13
 *                            (LM)
 *   value     uint16_t  I    new 15-bit flagword
 **********************************************************************/
void agc_flagword_set(int word, uint16_t value)
{
    g_flagword[word] = value;
}

/**********************************************************************
 * AGC: CMOONFLG / LMOONFLG (ERASABLE_ASSIGNMENTS C p. 61,
 *      FLAGWORD_ASSIGNMENTS L p. 79) -- the flag that names the primary
 *      body of the OTHER vehicle's permanent state vector
 *
 * FUNCTION NAME: other_moon_flag
 *
 * RETURNS: agc_flag_id_t -- LMOONFLG in the CSM, CMOONFLG in the LM
 **********************************************************************/
static agc_flag_id_t other_moon_flag(void)
{
    agc_flag_id_t f = (agc_flag_id_t) AGC_FLAG_LMOONFLG; /* CSM case */

    if (g_vehicle == AGC_VEHICLE_LM)
    {
        f = (agc_flag_id_t) AGC_FLAG_CMOONFLG;
    }

    return (f);
}

/**********************************************************************
 * AGC: RN, VN, PIPTIME, MOONFLAG (THIS) or RRECTOTH, VRECTOTH,
 *      TETOTHER and the other vehicle's moon flag (OTHER);
 *      ERASABLE_ASSIGNMENTS C pp. 79, 86, L pp. 105, 110-111
 *
 * FUNCTION NAME: agc_sv_get
 *
 * ARGUMENTS:
 *   Argument  Type                  I/O  Description (scale)
 *   --------  --------------------  ---  ---------------------------
 *   which     agc_vehicle_sv_t      I    AGC_SV_THIS or AGC_SV_OTHER
 *   out       agc_state_vector_t *  O    r (m, 2^29 earth / 2^27
 *                                        moon), v (m/cs, 2^7 / 2^5),
 *                                        t (cs, 2^28), moon flag
 **********************************************************************/
void agc_sv_get(agc_vehicle_sv_t which, agc_state_vector_t *out)
{
    if (which == AGC_SV_THIS)
    {
        out->r    = agc_erase_rn();
        out->v    = agc_erase_vn();
        out->t    = agc_erase_piptime();
        out->moon = agc_flag_test(AGC_FLAG_MOONFLAG);
    }
    else if (g_vehicle == AGC_VEHICLE_CSM)
    {
        out->r    = agc_erase_rrectlem(); /* RRECTOTH = RRECTLEM (C)  */
        out->v    = agc_erase_vrectlem();
        out->t    = agc_erase_tetlem();
        out->moon = agc_flag_test(other_moon_flag());
    }
    else
    {
        out->r    = agc_erase_rrectcsm(); /* RRECTOTH = RRECTCSM (L)  */
        out->v    = agc_erase_vrectcsm();
        out->t    = agc_erase_tetcsm();
        out->moon = agc_flag_test(other_moon_flag());
    }
}

/**********************************************************************
 * AGC: as agc_sv_get (ERASABLE_ASSIGNMENTS.agc C pp. 79, 86, L pp. 105,
 *      110-111) -- store a state vector and its moon flag
 *
 * FUNCTION NAME: agc_sv_set
 *
 * ARGUMENTS:
 *   Argument  Type                        I/O  Description (scale)
 *   --------  --------------------------  ---  ---------------------
 *   which     agc_vehicle_sv_t            I    AGC_SV_THIS or
 *                                              AGC_SV_OTHER
 *   in        const agc_state_vector_t *  I    r, v, t, moon at the
 *                                              scales of agc_sv_get
 *
 * RETURNS: agc_status_t -- AGC_STATUS_OK, or AGC_STATUS_BAD_ARG for a
 *          NULL in or bad which
 **********************************************************************/
agc_status_t agc_sv_set(agc_vehicle_sv_t which, const agc_state_vector_t *in)
{
    agc_flag_id_t moon_flag = (agc_flag_id_t) AGC_FLAG_MOONFLAG; /* THIS */

    if (which == AGC_SV_THIS)
    {
        agc_erase_set_rn(in->r);
        agc_erase_set_vn(in->v);
        agc_erase_set_piptime(in->t);
    }
    else if (g_vehicle == AGC_VEHICLE_CSM)
    {
        agc_erase_set_rrectlem(in->r);
        agc_erase_set_vrectlem(in->v);
        agc_erase_set_tetlem(in->t);
        moon_flag = other_moon_flag();
    }
    else
    {
        agc_erase_set_rrectcsm(in->r);
        agc_erase_set_vrectcsm(in->v);
        agc_erase_set_tetcsm(in->t);
        moon_flag = other_moon_flag();
    }

    agc_flag_put(moon_flag, in->moon); /* DEVIATION: see file prolog */

    if (!AGC_ASSERT(agc_flag_test(moon_flag) == in->moon))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
} /* agc_sv_set */
