/**********************************************************************
 * FILE NAME: test_erasables.c
 *
 * PURPOSE:
 *   The P07 test erasables W .. AINLA +99D as AGC words: fetched from
 *   the ECADR image when a job or task of the test starts and written
 *   back when it ends (job_entries.c), the SP / DP / word access the
 *   steps use, CCS, and the PIPA counter the test counts.
 *
 * AGC SOURCE:
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp.127-128 (W .. AINLA +99D,
 *   E5,1400); the CCS instruction
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors or by ECADR)
 *   Name          Type          I/O  Scale     Description
 *   ------------  ------------  ---  --------  ----------------------------
 *   W .. AINLA+99 uint16_t[]    I/O  words     the test erasables (st_words())
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   agc_erase_read_ecadr  the ECADR image (erasable_map)
 *   agc_erase_pipax ..    PIPA counters (state.h)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   None; AGC_STATUS_BAD_ARG from the write-back.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only; the jobs and tasks of P07 (V92).
 *
 * NOTES:
 *   DEVIATION (storage): the code depends on the erasables' layout
 *   (DATAPL overlays ANGZ, ALKCG stores ALTIM and ALTIMS with ALDK, LOSVEC
 *   +1 is NDXCTR), so they are kept as AGC words; see job_entries.c. **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/interp_words.h"
#include "agc/core/state.h"
#include "agc/erasable_map/address_map.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "internal.h"

/* The test erasables W .. AINLA +99D, as AGC words (see job_entries.c). */
static uint16_t w[W_WORDS]; /* W .. AINLA +99D as AGC words           */

/**********************************************************************
 * AGC: none -- the test erasables W .. AINLA +99D (W_WORDS AGC words),
 * for the step files of the test.
 *
 * ARGUMENTS: none
 *
 * RETURNS: the words (never NULL)
 **********************************************************************/
uint16_t *st_words(void)
{
    return (w);
}

/* ---- AGC words ---------------------------------------------------------- */

/**********************************************************************
 * AGC: none -- A ones'-complement word as a signed value (-0 reads as 0).
 *
 * ARGUMENTS:
 *   off  int  I   word offset from W
 *
 * RETURNS: the word's value
 **********************************************************************/
agc_sp_t w_sp(int off)
{
    uint16_t x = w[off]; /* the word as held */

    return ((agc_sp_t) (((x & 040000u) != 0u) ? -(int) (WORD15 - x) : (int) x)); /* negative: -(77777 - x) */
}

/**********************************************************************
 * AGC: none -- store a value as the AGC word at W +off
 *
 * ARGUMENTS:
 *   off  int       I   word offset from W
 *   v    agc_sp_t  I   value, |v| <= 16383 (-0 not stored)
 **********************************************************************/
void w_set_sp(int off, agc_sp_t v)
{
    w[off] = (uint16_t) ((v >= 0) ? (unsigned int) v : (unsigned int) ((int) WORD15 + v)); /* negative: 77777 + v */
}

/**********************************************************************
 * AGC: none -- the DP value held at W +off
 *
 * ARGUMENTS:
 *   off  int  I   offset of the major word from W
 *
 * RETURNS: the DP value of w[off], w[off +1] (2^-14 per minor unit)
 **********************************************************************/
agc_dp_t w_dp(int off)
{
    return (agc_dp_pack(w_sp(off), w_sp(off + 1)));
}

/**********************************************************************
 * AGC: none -- store a DP at W +off, W +off +1
 *
 * ARGUMENTS:
 *   off  int       I   offset of the major word from W
 *   v    agc_dp_t  I   DP value, stored as sign-agreeing words
 **********************************************************************/
void w_set_dp(int off, agc_dp_t v)
{
    w_set_sp(off, agc_dp_high(v));
    w_set_sp(off + 1, agc_dp_low(v));
}

/**********************************************************************
 * AGC: none -- the DP at W +off as the two words the AGC holds
 *
 * ARGUMENTS:
 *   off  int  I   offset of the major word from W
 *
 * RETURNS: the two words as held (signs may disagree)
 **********************************************************************/
agc_dp_words_t w_words(int off)
{
    agc_dp_words_t x = {w_sp(off), w_sp(off + 1)};

    return (x);
}

/**********************************************************************
 * AGC: none -- STORE: MPAC, MPAC +1 into w[off], w[off +1] as held.
 *
 * ARGUMENTS:
 *   off  int             I   offset of the major word from W
 *   m    agc_tp_words_t  I   MPAC; MPAC, MPAC +1 are stored
 **********************************************************************/
void w_store(int off, agc_tp_words_t m)
{
    agc_dp_words_t x = agc_mpac_store(m); /* MPAC, MPAC +1 as held */

    w_set_sp(off, x.hi);
    w_set_sp(off + 1, x.lo);
}

/**********************************************************************
 * AGC: none -- CCS on a word: the branch and what the AGC leaves in A
 *
 * ARGUMENTS:
 *   x  uint16_t  I   the word CCS tests
 *
 * RETURNS: the branch (0 > 0, 1 +0, 2 < 0, 3 -0) and A = |x| - 1
 **********************************************************************/
st_ccs_t ccs(uint16_t x)
{
    st_ccs_t c = {1, 0};

    if (x == WORD15)
    {
        c.branch = 3; /* -0 */
    }
    else if ((x & 040000u) != 0u)
    {
        c.branch = 2;
        c.a      = (uint16_t) (WORD15 - x - 1u);
    }
    else if (x != 0u)
    {
        c.branch = 0;
        c.a      = (uint16_t) (x - 1u);
    }

    return (c);
}

/**********************************************************************
 * AGC: none -- The PIPA counter PIPINDEX names (INDEX PIPINDEX, PIPAX).
 *
 * ARGUMENTS:
 *   axis  int *  O   PIPINDEX, 0..2 (X, Y, Z)
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG when PIPINDEX (ground loaded) is not
 *          0..2
 **********************************************************************/
agc_status_t pipa_index(int *axis)
{
    *axis = (int) w_sp(W_PIPINDEX);

    if (!AGC_ASSERT(*axis >= 0 && *axis <= 2))
    {
        return (AGC_STATUS_BAD_ARG); /* PIPINDEX is a ground-loaded word */
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- the PIPA counter INDEX PIPINDEX names (PIPAX +axis)
 *
 * ARGUMENTS:
 *   axis  int  I   0..2 (X, Y, Z)
 *
 * RETURNS: that PIPA counter, pulses
 **********************************************************************/
agc_sp_t pipa_get(int axis)
{
    agc_sp_t p = agc_erase_pipaz(); /* PIPAZ, pulses 2^0 */

    if (axis == 0)
    {
        p = agc_erase_pipax();
    }
    else if (axis == 1)
    {
        p = agc_erase_pipay();
    }

    return (p);
}

/**********************************************************************
 * AGC: none -- store into the PIPA counter INDEX PIPINDEX names
 *
 * ARGUMENTS:
 *   axis  int       I   0..2 (X, Y, Z)
 *   v     agc_sp_t  I   new count, pulses
 **********************************************************************/
void pipa_set(int axis, agc_sp_t v)
{
    if (axis == 0)
    {
        agc_erase_set_pipax(v);
    }
    else if (axis == 1)
    {
        agc_erase_set_pipay(v);
    }
    else
    {
        agc_erase_set_pipaz(v);
    }
}

/**********************************************************************
 * AGC: none -- A DP as the two words the ECADR image holds (TRANSM1, ALFDK).
 *
 * ARGUMENTS:
 *   ecadr  int               I   ECADR of the major word (an index from
 *                                ground-loaded words)
 *   v      agc_dp_words_t *  O   the two words as stored
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for an address outside erasable memory,
 *          or the image's read status
 **********************************************************************/
agc_status_t image_words(int ecadr, agc_dp_words_t *v)
{
    uint16_t hi = 0; /* major word as read */
    uint16_t lo = 0; /* minor word as read */

    if (!AGC_ASSERT(ecadr >= 0 && ecadr < (int) EC_ERASABLE))
    {
        return (AGC_STATUS_BAD_ARG); /* an index from ground-loaded words */
    }

    agc_erase_read_ecadr((uint16_t) ecadr, &hi);
    agc_erase_read_ecadr((uint16_t) (ecadr + 1), &lo);
    hi    = (uint16_t) (hi & WORD15);
    lo    = (uint16_t) (lo & WORD15);
    v->hi = (agc_sp_t) (((hi & 040000u) != 0u) ? -(int) (WORD15 - hi) : (int) hi); /* ones' complement */
    v->lo = (agc_sp_t) (((lo & 040000u) != 0u) ? -(int) (WORD15 - lo) : (int) lo); /* ones' complement */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- Fetch / write back the test erasables (see DEVIATION above).
 *
 * RETURNS: none
 **********************************************************************/
void st_fetch(void)
{
    for (int i = 0; i < W_WORDS; i++)
    {
        agc_erase_read_ecadr((uint16_t) (EC_W + (unsigned int) i), &w[i]);
        w[i] = (uint16_t) (w[i] & WORD15);
    }
}

/**********************************************************************
 * AGC: none -- write the test erasables back to the ECADR image
 *
 * RETURNS: AGC_STATUS_OK or the image's write status
 **********************************************************************/
agc_status_t st_flush(void)
{
    agc_status_t st = AGC_STATUS_OK; /* write status */

    for (int i = 0; i < W_WORDS && st == AGC_STATUS_OK; i++)
    {
        st = agc_erase_write_ecadr((uint16_t) (EC_W + (unsigned int) i), w[i]);
    }

    return (st);
}
