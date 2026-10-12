/**********************************************************************
 * FILE NAME: self_check.c
 *
 * PURPOSE:
 *   SELF-CHECK (the backup idle loop's test of the computer, option in
 *   SMODE, noun 27) and the bank sums of SHOW-BANKSUM (verb 91; the verb
 *   itself is in src/io/verbs/extended_verbs.c).
 *
 * AGC SOURCE:
 *   Comanche055 AGC_BLOCK_TWO_SELF-CHECK.agc pp.1394-1403
 *   Luminary099 AGC_BLOCK_TWO_SELF_CHECK.agc pp.1284-1293 (identical)
 *   (SMODECHK, SOPTIONS, ERRORS, SIDLOOP, ROPECHK, STSHOSUM, COMMFX,
 *   COMADRS, FXFX, FXADRS, ADSUM, ADRSCHK, CONTINU, NXTBNK, SOPTION,
 *   BNKCHK; SDISPLAY and ENDSUMS of EXTENDED_VERBS.agc C p.258 / L p.290)
 *
 * EXTERNAL VARIABLES:
 *   Name      Type      I/O  Scale  Description
 *   --------  --------  ---  -----  ----------------------------------
 *   SMODE     uint16_t  I/O  raw    self-check option (module storage)
 *   ERCOUNT   uint16_t  I/O  count  errors since the last SLAP1
 *   SCOUNT    uint16_t  O    count  SMODECHK passes, memory-test passes
 *   SKEEP1-3  uint16_t  O    raw    SHOW-BANKSUM display words
 *   SKEEP4    int       I/O  bank   bank being summed
 *
 * EXTERNAL REFERENCES:
 *   Name       Description
 *   ---------  ------------------------------------------------------
 *   agc_alarm  ALARM2 01102 (self-check malfunction)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   A bank whose sum is not +-bank number raises alarm 01102 and counts
 *   ERCOUNT (ERRORS). A + option then drops to the idle loop (SMODE =
 *   +0), a - option restarts at SELFCHK, -0 goes on with the next bank.
 *   BAD_ARG from the two entries that take caller data (the rope image
 *   and a bank number out of range).
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles. Runs only when no job is ready (DUMMYJOB -> ADVAN ->
 *   SELFRET); STARTSB2 sends it back to SELFCHK on every restart.
 *
 * NOTES:
 *   DEVIATION (memory tests): the C program has no erasable memory,
 *   counters or editing registers to exercise, so ERASCHK, CNTRCHK and
 *   CYCLSHFT pass by construction (they still count SCOUNT +1).
 *   DEVIATION (fixed memory): ROPECHK and SHOW-BANKSUM sum the rope image
 *   the simulator registers with agc_selfcheck_set_rope() (the real
 *   Comanche055/Luminary099 binary passes: tests/io/sysmisc.c). With no
 *   image the C program has no fixed memory to test: every bank passes
 *   and SHOW-BANKSUM sums +0 words.
 *   DEVIATION (timing): the AGC sums one word per CHECKNJ, giving way to
 *   every new job; agc_selfcheck_idle_pass sums one bank per idle pass.
 *   SCOUNT keeps the AGC's counting: one per SMODECHK (two per cycle of
 *   the "everything" options), SCOUNT +1 one per memory-test pass.
 *
 * ALGORITHM:
 *   A bank sum adds every word with end-around carry (ADSUM) until two
 *   consecutive "TC SELF" words (a word equal to its own address) and the
 *   word after them (the bugger word, chosen at assembly so the sum is
 *   +-bank number), or the last word of the bank.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"

#define WORD15           077777u                    /* one AGC word, 15 bits */
#define NEG0             077777u                    /* -0 */
#define NEG1             077776u                    /* -1 */
#define SIGN15           040000u                    /* BIT15, the sign bit */
#define LOW10            001777u                    /* LOW10: last relative address of a bank */
#define BANK_ADDR        002000u                    /* common fixed: 2000-3777 */
#define BANK2_ADDR       004000u                    /* SBIT12: fixed-fixed bank 02, 4000-5777 */
#define BANK3_ADDR       006000u                    /* SBNK03: fixed-fixed bank 03, 6000-7777 */
#define LSTBNKCH         043                        /* LSTBNKCH: last bank checked */
#define MAX_OPTION       8                          /* options 1..10 octal */
#define OPTION_ERASABLE  4                          /* SOPTION4: ERASCHK */
#define OPTION_ROPE      5                          /* SOPTION5: ROPECHK */
#define AT_SELFCHK       (-1)                       /* SELFRET = SELFCHK: no ROPECHK running */
#define ALARM_SELF_CHECK ((agc_alarm_code_t) 01102) /* self-check malfunction */

static const uint16_t *rope;                   /* the simulator's fixed memory, 15-bit words */
static int             rope_banks;             /* banks in the image */
static uint16_t        smode;                  /* SMODE, raw word (077777 = -0) */
static uint16_t        ercount;                /* ERCOUNT (ALMCADR +2) */
static uint16_t        scount[2];              /* SCOUNT, SCOUNT +1 */
static uint16_t        showsum[3];             /* SKEEP1..SKEEP3 as SDISPLAY leaves them */
static int             rope_bank = AT_SELFCHK; /* SKEEP4 while ROPECHK runs */
static int             show_bank;              /* SKEEP4 during SHOW-BANKSUM */

/**********************************************************************
 * AGC: none -- the simulator registers its fixed-memory image.
 *
 * ARGUMENTS:
 *   image   const uint16_t *  I  bank b at image[b * 1024], 15-bit
 *                                words; NULL with nbanks 0 = no rope
 *   nbanks  int               I  0..44 octal banks
 *
 * RETURNS: agc_status_t -- OK; BAD_ARG for a bank count out of range or
 *          a NULL image with banks
 **********************************************************************/
agc_status_t agc_selfcheck_set_rope(const uint16_t *image, int nbanks)
{
    if (!AGC_ASSERT(nbanks >= 0 && nbanks <= AGC_ROPE_BANKS) || !AGC_ASSERT(image != NULL || nbanks == 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    rope       = image;
    rope_banks = nbanks;

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (SUPDACAL / INDEX SKEEP3, CA 0000) -- one rope word.
 *
 * ARGUMENTS:
 *   bank  int  I  0..43 octal
 *   rel   int  I  relative address 0..1777 octal
 *
 * RETURNS: uint16_t -- the word, +0 beyond the registered image
 **********************************************************************/
static uint16_t rope_word(int bank, int rel)
{
    uint16_t w = 0; /* +0 outside the image */

    if (rope != NULL && bank < rope_banks)
    {
        w = (uint16_t) (rope[bank * AGC_ROPE_BANK_WORDS + rel] & WORD15);
    }

    return (w);
}

/**********************************************************************
 * AGC: none -- the 16-bit A register image of a word.
 *
 * ARGUMENTS:
 *   w  uint16_t  I  15-bit word
 *
 * RETURNS: uint32_t -- w with bit 15 copied into bit 16
 **********************************************************************/
static uint32_t to16(uint16_t w)
{
    return ((uint32_t) w | (((uint32_t) w & SIGN15) << 1));
}

/**********************************************************************
 * AGC: none (the AD instruction) -- 16-bit ones'-complement add.
 *
 * ARGUMENTS:
 *   a  uint32_t  I  16-bit A register
 *   b  uint32_t  I  16-bit operand
 *
 * RETURNS: uint32_t -- a + b with end-around carry, 16 bits
 **********************************************************************/
static uint32_t ad16(uint32_t a, uint32_t b)
{
    uint32_t s = a + b; /* 17-bit raw sum */

    if (s > 0177777u)
    {
        s = (s & 0177777u) + 1u; /* end-around carry */
    }

    return (s & 0177777u);
}

/**********************************************************************
 * ADSUM (p.1401): SKEEP1 = SKEEP1 + word. On overflow TS SKEEP1 stores
 * the overflow-corrected word, sets A = +-1 and skips the CAF S+ZERO, so
 * the carry is added back in (end-around at 15 bits).
 *
 * ARGUMENTS:
 *   sum   uint16_t  I  SKEEP1, the sum so far
 *   word  uint16_t  I  the word fetched (SKEEP2)
 *
 * RETURNS: uint16_t -- the new SKEEP1
 **********************************************************************/
static uint16_t adsum(uint16_t sum, uint16_t word)
{
    uint32_t a       = ad16(to16(sum), to16(word));                      /* AD SKEEP1 */
    uint16_t stored  = (uint16_t) ((a & 037777u) | ((a >> 1) & SIGN15)); /* TS SKEEP1 */
    bool     pos_ovf = (a & 0140000u) == 0040000u;                       /* A = +1 after TS */
    bool     neg_ovf = (a & 0140000u) == 0100000u;                       /* A = -1 after TS */
    uint16_t result  = stored;                                           /* CAF S+ZERO, AD, TS: no overflow */

    if (pos_ovf || neg_ovf)
    {
        uint32_t r = ad16(to16(pos_ovf ? 1u : NEG1), to16(stored)); /* AD SKEEP1 with A = +1 : -1 */

        result = (uint16_t) ((r & 037777u) | ((r >> 1) & SIGN15));
    }

    return (result);
}

/**********************************************************************
 * COMMFX / COMADRS / FXFX / FXADRS / ADSUM / ADRSCHK / CONTINU (pp.1400-
 * 1402): the sum of one bank. Banks 2 and 3 are read at their fixed-
 * fixed addresses (4000, 6000), all others through the bank window at
 * 2000; a "TC SELF" word is one equal to that address (CS word, AD
 * address leaves -0 in L).
 *
 * ARGUMENTS:
 *   bank    int         I  0..43 octal (callers: bounded loops, SKEEP4)
 *   sum     uint16_t *  O  SKEEP1, the raw sum (+-bank number when good)
 *   bugger  uint16_t *  O  SKEEP2, the last word summed
 **********************************************************************/
static void sum_bank(int bank, uint16_t *sum, uint16_t *bugger)
{
    uint16_t s      = 0;         /* SKEEP1 */
    int      tcself = 1;         /* SKEEP5: 1, 0 after one TC SELF, -1 after two */
    uint16_t word   = 0;         /* SKEEP2 */
    uint16_t base   = BANK_ADDR; /* SBIT11: common fixed */
    bool     done   = false;     /* SOPTION reached */

    if (bank == 2)
    {
        base = BANK2_ADDR; /* FXFX: SBIT12 */
    }
    else if (bank == 3)
    {
        base = BANK3_ADDR; /* FXFX: SBNK03 */
    }
    else
    {
        /* COMADRS: the bank window */
    }

    for (int rel = 0; rel < AGC_ROPE_BANK_WORDS && !done; rel++)
    {
        word = rope_word(bank, rel);
        s    = adsum(s, word);

        /* ADRSCHK: the last address, or the bugger word after two TC SELF. */
        done = (rel == (int) LOW10) || (tcself < 0);

        if (word == (uint16_t) (base + (uint16_t) rel))
        {
            tcself = (tcself > 0) ? 0 : -1; /* CCS SKEEP5: first : second TC SELF */
        }
        else
        {
            tcself = 1; /* CONTINU */
        }
    }

    *sum    = s;
    *bugger = word;
}

/**********************************************************************
 * AGC: none -- the bank sum of sum_bank for a caller-supplied bank.
 *
 * ARGUMENTS:
 *   bank    int         I  0..43 octal
 *   sum     uint16_t *  O  the raw sum
 *   bugger  uint16_t *  O  the bugger word
 *
 * RETURNS: agc_status_t -- OK; BAD_ARG for a bank out of range
 **********************************************************************/
agc_status_t agc_selfcheck_banksum(int bank, uint16_t *sum, uint16_t *bugger)
{
    if (!AGC_ASSERT(bank >= 0 && bank <= LSTBNKCH))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    sum_bank(bank, sum, bugger);

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * SOPT / BNKCHK (p.1403): force the sum to its absolute value (CCS
 * SKEEP1: -0 and +0 both give +0) and require |sum| = bank number
 * (-1CHK passes only on -1 = |sum| - bank - 1).
 *
 * ARGUMENTS:
 *   bank  int  I  0..43 octal (from a bounded loop)
 *
 * RETURNS: bool -- true when the bank sum is good (always with no rope:
 *          DEVIATION, file prolog)
 **********************************************************************/
static bool bank_good(int bank)
{
    uint16_t sum = 0; /* SKEEP1 */
    uint16_t bug = 0; /* bugger word (unused by ROPECHK) */

    sum_bank(bank, &sum, &bug);
    uint16_t mag = ((sum & SIGN15) != 0u) ? (uint16_t) (WORD15 - sum) : sum; /* |SKEEP1| */

    return (rope == NULL || mag == (uint16_t) bank);
}

/**********************************************************************
 * ROPECHK (p.1400), one bank per idle pass: BNKCHK, then ERRORS on a
 * bad sum, NXTBNK, and after the last bank ENDSUMS (SKEEP6 = -0) back to
 * SELFCHK.
 *
 * ERRORS (p.1397): ERCOUNT +1, ALARM2 01102; CCS SMODE: a + option goes
 * to SIDLOOP (SMODE = +0, SELFCHK), a - option restarts at SELFCHK, -0
 * continues with the next bank (TC SFAIL).
 **********************************************************************/
static void ropechk_bank(void)
{
    bool restart = false; /* TC SELFCHK after an error */

    if (!bank_good(rope_bank))
    {
        ercount = (uint16_t) ((ercount + 1u) & WORD15); /* INCR ERCOUNT */
        agc_alarm(ALARM_SELF_CHECK);                    /* ALARM2 01102 */
        restart = (smode != NEG0);                      /* CCS SMODE: -0 goes on (TC SFAIL) */

        if ((smode & SIGN15) == 0u)
        {
            smode = 0; /* SIDLOOP: a + option */
        }
    }

    rope_bank = (restart || rope_bank >= LSTBNKCH) ? AT_SELFCHK : rope_bank + 1; /* SELFCHK / ENDSUMS : NXTBNK */
}

/**********************************************************************
 * SMODECHK / SOPTIONS (pp.1397-1398) at SELFCHK: SCOUNT +1 per SMODECHK
 * with a legal option; > 10 octal goes to the idle loop (BNKOPTN). Option
 * 4 runs ERASCHK..CYCLSHFT only, 5 ROPECHK only, the others and -0 both.
 *
 * ARGUMENTS:
 *   option  int  I  |SMODE| (0 for -0), not +0
 **********************************************************************/
static void smodechk(int option)
{
    if (option > MAX_OPTION)
    {
        smode = 0; /* BNKOPTN: SIDLOOP */
    }
    else
    {
        scount[0] = (uint16_t) ((scount[0] + 1u) & WORD15); /* INCR SCOUNT */

        if (option != OPTION_ROPE)
        {
            /* ERASCHK, CNTRCHK, CYCLSHFT (pass by construction), then
             * INCR SCOUNT +1 and SMODECHK again before ROPECHK.         */
            scount[1] = (uint16_t) ((scount[1] + 1u) & WORD15);

            if (option != OPTION_ERASABLE)
            {
                scount[0] = (uint16_t) ((scount[0] + 1u) & WORD15); /* INCR SCOUNT */
            }
        }

        if (option != OPTION_ERASABLE)
        {
            rope_bank = 0; /* ROPECHK: SKEEP6 = -0, STSHOSUM: SKEEP4 = bank 0 */
        }
    }
}

/**********************************************************************
 * ADVAN -> SELFRET (EXECUTIVE.agc C p.1220 / L p.1116): one pass of the
 * self-check when no job is ready. +0 in SMODE is the backup idle loop.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_selfcheck_idle_pass(void)
{
    if (rope_bank != AT_SELFCHK)
    {
        ropechk_bank();
    }
    else if (smode != 0u)
    {
        int option = 0; /* -0: same as 10 octal until an error */

        if (smode != NEG0)
        {
            option = (int) (((smode & SIGN15) != 0u) ? (WORD15 - smode) : smode); /* |SMODE|: - option : + option */
        }

        smodechk(option);
    }
    else
    {
        /* SMODE = +0: the backup idle loop. */
    }
}

/**********************************************************************
 * AGC: none (noun 27 load, PINBALL_NOUN_TABLES.agc) -- SMODE (any word;
 * > 10 octal reaches the idle loop at the next SMODECHK).
 *
 * ARGUMENTS:
 *   word  uint16_t  I  raw 15-bit word (077777 = -0)
 **********************************************************************/
void agc_selfcheck_set_smode(uint16_t word)
{
    smode = (uint16_t) (word & WORD15);
}

/**********************************************************************
 * AGC: none (noun 27 read) -- SMODE.
 *
 * RETURNS: uint16_t -- SMODE, raw word (077777 = -0)
 **********************************************************************/
uint16_t agc_selfcheck_smode(void)
{
    return (smode);
}

/**********************************************************************
 * DOFSTART (FRESH_START_AND_RESTART.agc C p.184 / L p.212): SMODE =
 * ERESTORE = +0; SLAP1 (C p.183 / L p.211) also clears ERCOUNT.
 *
 * ARGUMENTS:
 *   slap1  bool  I  true for the V36 SLAP1 fresh start
 **********************************************************************/
void agc_selfcheck_fresh_start(bool slap1)
{
    smode = 0;

    if (slap1)
    {
        ercount = 0;
    }
}

/**********************************************************************
 * STARTSB2 (FRESH_START_AND_RESTART.agc C p.188 / L p.215): CAF LESCHK,
 * TS SELFRET -- every restart sends the self-check back to SELFCHK.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_selfcheck_restart(void)
{
    rope_bank = AT_SELFCHK;
}

/**********************************************************************
 * AGC: none (noun 08 R3) -- ERCOUNT, ALMCADR +2.
 *
 * RETURNS: uint16_t -- self-check errors since the last SLAP1
 **********************************************************************/
uint16_t agc_selfcheck_error_count(void)
{
    return (ercount);
}

/**********************************************************************
 * SOPTION -> SDISPLAY (EXTENDED_VERBS.agc C p.258 / L p.290) for
 * SHOW-BANKSUM: SKEEP1 = the sum, SKEEP2 = the bank number (LXCH SKEEP2),
 * SKEEP3 = the bugger word (LXCH SKEEP3).
 *
 * ARGUMENTS:
 *   bank  int  I  0..43 octal (STSHOSUM / NXTBNK)
 **********************************************************************/
static void showsum_bank(int bank)
{
    uint16_t sum = 0; /* SKEEP1 */
    uint16_t bug = 0; /* bugger word */

    sum_bank(bank, &sum, &bug);
    show_bank  = bank;
    showsum[0] = sum;
    showsum[1] = (uint16_t) bank;
    showsum[2] = bug;
}

/**********************************************************************
 * SHOWSUM (EXTENDED_VERBS.agc C p.258 / L p.290): SKEEP6 = +1 (the
 * SHOWSUM option: here, this entry), SMODE = +0 (self-check off),
 * SELFRET = SELFCHK, then STSHOSUM sums bank 0. V34 on the display
 * stores SELFADRS in SKEEP1, the self-check's return, which SELFRET
 * already gives here: nothing to do.
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_selfcheck_showsum_start(void)
{
    smode     = 0;
    rope_bank = AT_SELFCHK;
    showsum_bank(0);
}

/**********************************************************************
 * NXTBNK (AGC_BLOCK_TWO_SELF-CHECK.agc p.1402; V33 on the display);
 * after the last bank ENDSUMS (SKEEP6 = +1) starts again at bank 0
 * (STSHOSUM).
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_selfcheck_showsum_next(void)
{
    int next = (show_bank >= LSTBNKCH) ? 0 : show_bank + 1; /* ENDSUMS : NXTBNK */

    showsum_bank(next);
}

/**********************************************************************
 * AGC: none (SDISPLAY's V05 N01 at SKEEP1) -- the three words shown.
 *
 * ARGUMENTS:
 *   words  uint16_t[3]  O  SKEEP1 (sum), SKEEP2 (bank), SKEEP3 (bugger)
 **********************************************************************/
void agc_selfcheck_showsum_words(uint16_t words[3])
{
    for (int k = 0; k < 3; k++)
    {
        words[k] = showsum[k];
    }
}
