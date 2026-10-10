/*
 * self_check.c -- AGC_BLOCK_TWO_SELF-CHECK.agc (Comanche055 pp. 1394-1403,
 * Luminary099 pp. 1284-1293; the two are the same code).
 *
 * SELF-CHECK runs in the backup idle loop with the option in SMODE (N27):
 *   +-4 erasable memory (ERASCHK, CNTRCHK, CYCLSHFT), +-5 fixed memory
 *   (ROPECHK), +-1,2,3,6,7,10 both, -0 both until an error, +0 none.
 * An error stores the failure, counts ERCOUNT, raises alarm 01102 and, for
 * a + option, drops to the idle loop (SMODE = +0).
 * SHOW-BANKSUM (V91, extended_verbs.c) shows bank by bank R1 = the bank
 * sum (+-bank number when good), R2 = the bank number, R3 = the bugger
 * word.
 *
 * DEVIATION (memory tests): the C program has no erasable memory, counters
 * or editing registers to exercise, so ERASCHK, CNTRCHK and CYCLSHFT pass
 * by construction (they still count SCOUNT +1).  The fixed-memory test is
 * real: it runs over the rope image the simulator registers with
 * agc_selfcheck_set_rope() (no image = all words +0).
 *
 * DEVIATION (timing): the AGC sums one word per pass through CHECKNJ /
 * CHANG1, giving way to every new job; here one call sums a whole bank.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service_internal.h"

#define WORD15           077777u
#define NEG0             077777u /* -0                                    */
#define NEG1             077776u /* -1                                    */
#define SIGN15           040000u
#define LOW10            001777u
#define BANK_ADDR        002000u /* common fixed: 2000-3777              */
#define BANK2_ADDR       004000u /* fixed-fixed bank 02: 4000-5777        */
#define BANK3_ADDR       006000u /* fixed-fixed bank 03: 6000-7777        */
#define LSTBNKCH         043     /* last bank checked                     */
#define MAX_OPTION       8       /* options 1..10 octal                   */
#define ALARM_SELF_CHECK ((agc_alarm_code_t) 01102)

static const uint16_t *rope; /* the simulator's fixed memory       */
static int             rope_banks;
static uint16_t        smode;     /* SMODE, raw word (077777 = -0)      */
static uint16_t        ercount;   /* ERCOUNT                            */
static uint16_t        scount[2]; /* SCOUNT, SCOUNT +1                  */
static uint16_t        skeep[8];  /* SKEEP1..SKEEP7 (index 1..7)        */
static int             show_bank; /* SKEEP4 bank during SHOW-BANKSUM    */

agc_status_t agc_selfcheck_set_rope(const uint16_t *image, int nbanks)
{
    if (!AGC_ASSERT(nbanks >= 0 && nbanks <= AGC_ROPE_BANKS) ||
        !AGC_ASSERT(image != NULL || nbanks == 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    rope       = image;
    rope_banks = nbanks;

    return AGC_STATUS_OK;
}

/* Word rel of bank b, +0 outside the image.                                */
static uint16_t rope_word(int bank, int rel)
{
    if (!AGC_ASSERT(bank >= 0 && bank <= LSTBNKCH))
    {
        return 0; /* no valid result */
    }

    if (!AGC_ASSERT(rel >= 0 && rel < AGC_ROPE_BANK_WORDS))
    {
        return 0; /* no valid result */
    }

    if (rope == NULL || bank >= rope_banks)
    {
        return 0;
    }

    return (uint16_t) (rope[bank * AGC_ROPE_BANK_WORDS + rel] & WORD15);
}

/* 16-bit ones'-complement A register image of a 15-bit word.               */
static uint32_t to16(uint16_t w)
{
    if (!AGC_ASSERT(w <= WORD15))
    {
        return 0; /* no valid result */
    }

    return (uint32_t) w | (((uint32_t) w & SIGN15) << 1);
}

/* AD in the 16-bit accumulator (end-around carry).                         */
static uint32_t ad16(uint32_t a, uint32_t b)
{
    if (!AGC_ASSERT(a <= 0177777u && b <= 0177777u))
    {
        return 0; /* no valid result */
    }

    uint32_t s = a + b;

    if (s > 0177777u)
    {
        s = (s & 0177777u) + 1u;
    }

    return s & 0177777u;
}

/* ADSUM: SKEEP1 = SKEEP1 + word; on overflow TS stores the overflow-
 * corrected word and A = +-1 is added back in (end-around).               */
static uint16_t adsum(uint16_t sum, uint16_t word)
{
    uint32_t a       = ad16(to16(sum), to16(word));
    uint16_t stored  = (uint16_t) ((a & 037777u) | ((a >> 1) & SIGN15)); /* TS */
    bool     pos_ovf = (a & 0140000u) == 0040000u;
    bool     neg_ovf = (a & 0140000u) == 0100000u;

    if (!pos_ovf && !neg_ovf)
    {
        return stored; /* CAF S+ZERO, AD, TS */
    }

    uint32_t r = ad16(to16(pos_ovf ? 1u : NEG1), to16(stored));

    if (!AGC_ASSERT((r & 0140000u) == 0u || (r & 0140000u) == 0140000u))
    {
        return 0; /* no valid result */
    }

    return (uint16_t) ((r & 037777u) | ((r >> 1) & SIGN15));
}

/* First address of bank b as the self-check forms it.                     */
static uint16_t bank_base(int bank)
{
    if (!AGC_ASSERT(bank >= 0 && bank <= LSTBNKCH))
    {
        return 0; /* no valid result */
    }

    return (bank == 2) ? BANK2_ADDR : (bank == 3) ? BANK3_ADDR : BANK_ADDR;
}

/* COMADRS/FXADRS .. ADRSCHK: sum bank b up to the second of two
 * consecutive TC SELF words and the bugger word after them (or the last
 * word).                                                                   */
agc_status_t agc_selfcheck_banksum(int bank, uint16_t *sum, uint16_t *bugger)
{
    if (!AGC_ASSERT(sum != NULL && bugger != NULL) ||
        !AGC_ASSERT(bank >= 0 && bank <= LSTBNKCH))
    {
        return AGC_STATUS_BAD_ARG;
    }

    uint16_t s      = 0; /* SKEEP1              */
    int      tcself = 1; /* SKEEP5: 1, 0, -1    */
    uint16_t word   = 0; /* SKEEP2              */

    for (int rel = 0; rel < AGC_ROPE_BANK_WORDS; rel++)
    {
        word = rope_word(bank, rel);
        s    = adsum(s, word);

        if (rel == (int) LOW10 || tcself < 0)
        {
            break; /* SOPTION             */
        }

        uint16_t addr = (uint16_t) (bank_base(bank) + (uint16_t) rel);

        if (word == addr)
        { /* L = -0: a TC SELF   */
            tcself = (tcself > 0) ? 0 : -1;
        }
        else
        {
            tcself = 1; /* CONTINU             */
        }
    }

    *sum    = s;
    *bugger = word;

    return AGC_STATUS_OK;
}

/* BNKCHK: |sum| must equal the bank number.                                */
bool agc_selfcheck_bank_ok(int bank)
{
    uint16_t     sum = 0;
    uint16_t     bug = 0;
    agc_status_t st  = agc_selfcheck_banksum(bank, &sum, &bug);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return false;
    }

    uint16_t mag = ((sum & SIGN15) != 0u) ? (uint16_t) (WORD15 - sum) : sum;

    return mag == (uint16_t) bank;
}

/* ERRORS: count, alarm 01102; a + option drops to the idle loop.          */
static agc_status_t selfcheck_error(void)
{
    ercount = (uint16_t) ((ercount + 1u) & WORD15);
    agc_alarm(ALARM_SELF_CHECK); /* ALARM2 01102       */

    if (smode != NEG0 && (smode & SIGN15) == 0u)
    {
        smode = 0; /* SIDLOOP            */
    }

    return AGC_STATUS_ALARM;
}

/* ROPECHK: every bank 0..43 in order.                                     */
static agc_status_t ropechk(void)
{
    skeep[6] = NEG0; /* -0: ROPECHK         */

    for (int b = 0; b <= LSTBNKCH; b++)
    {
        if (!agc_selfcheck_bank_ok(b))
        {
            skeep[4] = (uint16_t) b;

            return selfcheck_error();
        }
    }

    if (!AGC_ASSERT(skeep[6] == NEG0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

/* SMODECHK + the selected option, one complete cycle (SELFCHK entry).    */
agc_status_t agc_selfcheck_idle_pass(void)
{
    if (smode == 0u)
    {
        return AGC_STATUS_OK; /* backup idle loop    */
    }

    int option =
        (smode == NEG0) ?
        0 :
        (int) (((smode & SIGN15) != 0u) ? (WORD15 - smode) : smode);

    if (option > MAX_OPTION)
    {
        smode = 0; /* BNKOPTN: SIDLOOP    */

        return AGC_STATUS_OK;
    }

    scount[0] = (uint16_t) ((scount[0] + 1u) & WORD15);

    if (option != 5)
    { /* ERASCHK..CYCLSHFT   */
        scount[1] = (uint16_t) ((scount[1] + 1u) & WORD15);
    }

    if (option == 4)
    {
        return AGC_STATUS_OK;
    }

    return ropechk();
}

agc_status_t agc_selfcheck_set_smode(uint16_t word)
{
    if (!AGC_ASSERT(word <= WORD15))
    {
        return AGC_STATUS_BAD_ARG;
    }

    smode = word;

    return AGC_STATUS_OK;
}

uint16_t agc_selfcheck_smode(void)
{
    if (!AGC_ASSERT(smode <= WORD15))
    {
        return 0; /* no valid result */
    }

    return smode;
}

/* DOFSTART: SMODE = ERESTORE = +0; SLAP1 also clears ERCOUNT.            */
agc_status_t agc_selfcheck_fresh_start(bool slap1)
{
    smode = 0;

    if (slap1)
    {
        ercount = 0;
    }

    if (!AGC_ASSERT(smode == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

uint16_t agc_selfcheck_error_count(void)
{
    if (!AGC_ASSERT(ercount <= WORD15))
    {
        return 0; /* no valid result */
    }

    return ercount;
}

/* SOPTION / SDISPLAY for SHOW-BANKSUM: SKEEP1 sum, SKEEP2 bank, SKEEP3
 * bugger word.                                                            */
static agc_status_t showsum_bank(int bank)
{
    uint16_t     sum = 0;
    uint16_t     bug = 0;
    agc_status_t st  = agc_selfcheck_banksum(bank, &sum, &bug);

    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }

    show_bank = bank;
    skeep[1]  = sum;
    skeep[2]  = (uint16_t) bank;
    skeep[3]  = bug;

    return AGC_STATUS_OK;
}

/* SHOWSUM: SKEEP6 = +1, SMODE = +0 (self-check off), STSHOSUM.           */
agc_status_t agc_selfcheck_showsum_start(void)
{
    skeep[6] = 1;
    smode    = 0;

    if (!AGC_ASSERT(skeep[6] == 1u))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return showsum_bank(0);
}

/* NXTBNK; after the last bank ENDSUMS starts again at bank 0.            */
agc_status_t agc_selfcheck_showsum_next(void)
{
    if (!AGC_ASSERT(show_bank >= 0 && show_bank <= LSTBNKCH))
    {
        return AGC_STATUS_BAD_ARG;
    }

    int next = (show_bank >= LSTBNKCH) ? 0 : show_bank + 1;

    return showsum_bank(next);
}

/* V34: SKEEP1 = SELFADRS (self-check restarts at SELFCHK).               */
agc_status_t agc_selfcheck_showsum_end(void)
{
    skeep[1] = 0;
    skeep[6] = NEG0;

    if (!AGC_ASSERT(show_bank >= 0))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

uint16_t agc_selfcheck_skeep(int i)
{
    if (!AGC_ASSERT(i >= 1 && i <= 7))
    {
        return 0;
    }

    return skeep[i];
}
