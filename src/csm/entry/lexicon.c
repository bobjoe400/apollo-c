/**********************************************************************
 * FILE NAME: lexicon.c
 *
 * PURPOSE:
 *   The entry erasable set (the ENTRY_LEXICON names, one module-private
 *   structure reached through agc_entry_state()) and the sub-orbital
 *   reference trajectory table VREFER used by PREDICT3.
 *
 * AGC SOURCE:
 *   Comanche055 ENTRY_LEXICON.agc pp. 837-843   (erasable assignments)
 *   Comanche055 REENTRY_CONTROL.agc pp. 876-878 (VREFER .. LND/DT tables)
 *   Luminary099: none (CM only)
 *
 * EXTERNAL VARIABLES:
 *   none (the entry erasables are this file's own structure)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   memset                fresh-start zeroing of the erasable set
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; an out-of-range VREFER index asserts and reads 0.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   CSM only. One entry erasable set (EBANK6/7 in the AGC).
 *
 * NOTES:
 *   The seven 13-point tables are stored back to back as in the rope, so
 *   PREDICT3's flat index (JJ + 13 per table, and "VREFER +1" with
 *   JJ = 12 reading the next table) is kept.
 **********************************************************************/
#include <string.h>

#include "agc/core/assert.h"
#include "agc/csm/entry.h"
#include "agc/csm/entry_lexicon.h"

/* The entry erasables. EBANK7 (RTINIT..) and EBANK6 (AOG..) in the AGC.    */
static agc_entry_state_t entry_state; /* scales per member */

/* VREFER: 13 reference velocities (V/2VS) followed by six 13-point
 * functions of V, SP. Stored consecutively exactly as in the AGC so the
 * PREDICT3 index arithmetic (JJ + 13 per table) is preserved.             */
static const agc_sp_t entry_vrefer_table[AGC_ENTRY_VREFER_TABLES *
                                         AGC_ENTRY_VREFER_N] = {
    /* table 0: VREF, V/51532.3946 (2VS) */
    /* (the last point is "high velocity for safety")                  */
    316, 669, 1247, 2001, 2712, 3211, 4456, 5071, 5836, 6622, 7341, 7471, 11128,

    /* AGC: .019288 .040809 .076107 .122156 .165546 .196012 .271945 .309533
     * .356222 .404192 .448067 .456023 .67918 */
    /* table 1: DRANGE/DA (DRDA/(2700/805)) */
    -169, -271, -441, -689, -966, -1159, -1614, -1761, -2421, -3167, -9872,
    -16383, -16383,

    /* AGC: -.010337 -.016550 -.026935 -.042039 -.058974 -.070721 -.098538
     * -.107482 -.147762 -.193289 -.602557 -.99999 -.99999 */
    /* table 2: -DRANGE/DRDOT ((2VS/8 2700) DR/DRDOT) */
    -98, -140, -275, -565, -969, -1325, -2400, -3003, -3902, -5218, -8502,
    -11906, -11906,

    /* AGC: -.0478599 B-3 -.0683663 B-3 -.1343468 B-3 -.2759846 B-3 -.4731437
     * B-3 -.6472087 B-3 -1.171693 B-3 -1.466382 B-3 -1.905171 B-3 -2.547990
     * B-3 -4.151220 B-3 -5.813617 B-3 -5.813617 B-3 */
    /* table 3: RDOTREF (8 RDT/2VS) */
    -1756, -1828, -1764, -1548, -1253, -1058, -895, -1058, -1439, -1986, -2357,
    -2085, -2085,

    /* AGC: -.0134001  B3 -.013947   B3 -.013462   B3 -.011813   B3 -.0095631
     * B3 -.00806946 B3 -.006828   B3 -.00806946 B3 -.0109791  B3 -.0151498
     * B3 -.0179817  B3 -.0159061  B3 -.0159061  B3 */
    /* table 4: RTOGO (RTOGO/2700) */
    13, 54, 134, 281, 458, 606, 1037, 1276, 1619, 2089, 3063, 3902, 4820,

    /* AGC: .0008067 .0032963 .0081852 .017148 .027926 .037 .063296 .077889
     * .098815 .127519 .186963 .238148 .294185185 */
    /* table 5: -AREF/805 */
    -837, -1221, -1659, -1911, -2005, -2082, -2416, -2548, -2450, -1942, -572,
    -130, -130,

    /* AGC: -.051099 -.074534 -.101242 -.116646 -.122360 -.127081 -.147453
     * -.155528 -.149565 -.118509 -.034907 -.007950 -.007950 */
    /* table 6: DRANGE/D L/D (Y/2700) */
    74, 132, 263, 587, 1137, 1712, 1999, 2825, 4143, 5950, 8404, 9151, 9151,

    /* AGC: .004491 .008081 .016030 .035815 .069422 .104519 .122 .172407
     * .252852 .363148 .512963 .558519 .558519 */
};

/**********************************************************************
 * AGC: none (ENTRY_LEXICON.agc p. 837 names) -- the entry erasable set
 *
 * FUNCTION NAME: agc_entry_state
 *
 * ARGUMENTS: none
 *
 * RETURNS: agc_entry_state_t * -- the single entry erasable set (never
 *          NULL)
 **********************************************************************/
agc_entry_state_t *agc_entry_state(void)
{
    agc_entry_state_t *es = &entry_state; /* the one erasable set     */

    if (!AGC_ASSERT((int) es->gotoaddr >= 0))
    {
        es->gotoaddr = AGC_ENTRY_GOTO_INITROLL; /* recover: restart seq. */
    }

    if (!AGC_ASSERT(es->gotoaddr < AGC_ENTRY_GOTO_COUNT))
    {
        es->gotoaddr = AGC_ENTRY_GOTO_INITROLL;
    }

    return (es);
}

/**********************************************************************
 * AGC: none (fresh start, ENTRY_LEXICON.agc p. 837) -- reset the set
 *
 * Fresh-start contents (all erasable zero), sequencer at INITROLL, CM/POSE
 * exit at P62.3 (its first use in P62).
 *
 * FUNCTION NAME: agc_entry_lexicon_reset
 *
 * ARGUMENTS: none
 *
 * RETURNS: none
 **********************************************************************/
void agc_entry_lexicon_reset(void)
{
    (void) memset(&entry_state, 0, sizeof (entry_state)); /* returns its arg */
    entry_state.gotoaddr = AGC_ENTRY_GOTO_INITROLL;
    entry_state.posexit  = AGC_ENTRY_POSEXIT_DESIRED_GIMBALS;
    entry_state.cmdapmod = AGC_ENTRY_DAPMOD_PLUS0;

    if (!AGC_ASSERT(entry_state.jj == 0))
    {
        return;
    }

    if (!AGC_ASSERT(entry_state.d == 0))
    {
        return;
    }
}

/**********************************************************************
 * VREFER (REENTRY_CONTROL.agc pp. 876-878) -- one table word
 *
 * VREFER + 13 table + k, the AGC's flat indexing (k may be 13: "VREFER
 * +1" with JJ = 12 reads the next table, as in the AGC).
 *
 * FUNCTION NAME: agc_entry_vrefer
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------------
 *   table     int   I    table number 0..6 (VREF .. DRANGE/D L/D)
 *   k         int   I    point index, 0..12 (13 reads the next table)
 *
 * RETURNS: agc_sp_t -- the table word at the scale of that table (see the
 *          table comments); 0 when the flat index is out of range
 **********************************************************************/
agc_sp_t agc_entry_vrefer(int table, int k)
{
    int flat = (table * AGC_ENTRY_VREFER_N) + k; /* VREFER offset    */

    if (!AGC_ASSERT(table >= 0 && table < AGC_ENTRY_VREFER_TABLES))
    {
        return (0);
    }

    if (!AGC_ASSERT(flat >= 0 && flat < (AGC_ENTRY_VREFER_TABLES * AGC_ENTRY_VREFER_N)))
    {
        return (0);
    }

    return (entry_vrefer_table[flat]);
}
