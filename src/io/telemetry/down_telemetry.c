/**********************************************************************
 * FILE NAME: down_telemetry.c
 *
 * PURPOSE:
 *   The down-telemetry program: every DOWNRUPT (20 ms) one DP word of the
 *   selected downlist (ID / sync pair, then the words its control list and
 *   sublists name) to channels 34 and 35, the word order code in channel
 *   13; the V74 erasable dump in place of the downlist.
 *
 * AGC SOURCE:
 *   Comanche055 DOWN-TELEMETRY_PROGRAM.agc pp.1093-1102
 *   Luminary099 DOWN_TELEMETRY_PROGRAM.agc pp.988-997 (WOTEST only)
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached by address, agc_erase_read_ecadr)
 *   Name          Type      I/O  Scale  Description
 *   ------------  --------  ---  -----  ----------------------------------
 *   the downlist  uint16_t  I    words  every word the lists name, by ECADR
 *   words
 *   DNTMBUFF      uint16_t  I/O  words  the snapshot buffer (0340, both ropes)
 *   TIME1         uint16_t  I    cs     sent after DUMPLOC
 *   CHAN13        uint16_t  I/O  bits   bit 7: word order code
 *   DUMPLOC words uint16_t  I    words  the dumped registers (ECADR image)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_dnlist_*           the list tables (downlink_lists_{csm,lm}.c)
 *   agc_erase_read_ecadr   FETCH2WD, SNAPLOOP, DNDUMP2: a word by address
 *   agc_erase_write_ecadr  SNAPLOOP: the snapshot into DNTMBUFF
 *   agc_chan_*             channels 13, 34, 35
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   none; a DNLSTCOD outside DNTABLE is refused (BAD_ARG) at NEWLIST.
 *
 * NOTES:
 *   Every DOWNRUPT (20 ms) the program writes one DP downlink word: the
 *   high SP word to channel 34 (DNTM1) and the low one to channel 35
 *   (DNTM2).  A downlist is 100 such words (2 s): the ID / sync pair,
 *   then the words named by the control list and its sublists
 *   (src/io/telemetry/downlink_lists_{csm,lm}.c).  Channel 13 bit 7
 *   (word order code) is 0 for the ID word and for TIME2,TIME1 and 1
 *   otherwise.  V74 replaces the downlist by an erasable dump
 *   (DNDUMPI/DNDUMP), which ends after DUMPCNT complete dumps and
 *   resumes the interrupted downlist at DNPHASE1.
 *
 *   The two vehicles differ only in DODOWNTM: Luminary sets the word
 *   order bit only when it is not already set (WOTEST); Comanche always
 *   WORs it. The effect is the same and is modelled once.
 *
 *   DEVIATION (DNTMGOTO): the AGC keeps the address of the next phase in
 *   DNTMGOTO (DNPHASE1, DNPHASE2, DNDUMPI +3, DNDUMP1, DNDUMP); here it
 *   is an enum.  DNECADR (count and ECADR of the nDNADR in progress),
 *   CTLIST and SUBLIST (list pointers, negative at the last entry)
 *   become an item id, word offset and word count plus list indices with
 *   "last entry" flags.
 *
 *   Data words: as in the AGC, a list entry is the ECADR of the words it
 *   sends, and FETCH2WD reads them by address (agc_erase_read_ecadr,
 *   src/erasable_map: the address column at the top of the layering, which
 *   reaches each erasable in its owner's storage); the snapshot goes into
 *   DNTMBUFF by address too, so DNTMBUFF is erasable as in the AGC.  The
 *   erasable dump reads its words the same way.
 *
 *   DUMPCNT is set by SLAP1 (LM fresh start, one dump).  The V21N01 load
 *   of DUMPCNT the AGC allows goes to the address image and does not
 *   reach it (same DEVIATION); with DUMPCNT = 0 the AGC, and this code,
 *   dump forever.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/erasable_map/address_map.h"
#include "agc/io/service.h"

#define WORD15       077777u /* one AGC word */
#define BIT7         000100u /* CHAN13 bit 7: downlink word order code   */
#define LOW8         000377u /* relative address in an EBANK */
#define PRIO34       034000u /* DUMPLOC dump counter, bits 12-14         */
#define ERASID_CSM   001777u /* ERASID EQUALS LOW10 (Comanche)           */
#define ERASID_LM    001776u /* ERASID EQUALS BITS2-10 (Luminary)        */
#define ADR_DNTMBUFF 00340u  /* DNTMBUFF ERASE +21D (both ropes, 0340)   */
#define ADR_TIME2    00024u  /* TIME2 (register 024); TIME1 at 025       */
#define ADR_TIME1    00025u  /* TIME1 (register 025)                     */
#define ECADR_MASK   003777u /* DUMPLOC EEE RRRRRRRR: the ECADR dumped   */

/* DNTMGOTO: where the next DOWNRUPT goes.                                 */
typedef enum tm_phase
{
    TM_PHASE1 = 0, /* DNPHASE1: restart the current downlist          */
    TM_PHASE2,     /* DNPHASE2: send the next downlist word           */
    TM_DUMP_ID,    /* DNDUMPI +2: send the dump ID word               */
    TM_DUMP_LOC,   /* DNDUMPI +3: send DUMPLOC and TIME1              */
    TM_DUMP_FIRST, /* DNDUMP1: first two words of an EBANK            */
    TM_DUMP_NEXT   /* DNDUMP: the following words                     */
} tm_phase_t;

/* Where the DOWNRUPT left a list: one list (control or sublist), the
 * index of the next entry, and whether the last entry has been taken
 * (CTLIST / SUBLIST negative).                                           */
typedef struct tm_cursor
{
    agc_dn_list_t list;   /* the control list or sublist */
    int           next;   /* index of the next entry */
    bool          active; /* SUBLIST positive / CTLIST positive        */
} tm_cursor_t;

static tm_phase_t  dntmgoto = TM_PHASE1; /* DNTMGOTO, the next phase */
static int         dnlstcod;             /* DNLSTCOD, B(1)PRM list code      */
static uint16_t    agsword;              /* AGSWORD (LM): saved DNLSTCOD     */
static uint16_t    dumpcnt;              /* DUMPCNT, B(1)PRM                 */
static uint16_t    dumploc;              /* DUMPLOC: dump counter + ECADR    */
static tm_cursor_t ctlist;               /* CTLIST                           */
static tm_cursor_t sublist;              /* SUBLIST                          */
static uint16_t    dn_ecadr;             /* DNECADR: next ECADR of the nDNADR */
static int         dn_left;              /*   DP words still to send (> 0)   */

/* ---- words by address ------------------------------------------------- */

/**********************************************************************
 * AGC: none -- the word at an ECADR (FETCH2WD's EBANK, DCA 1400 +R)
 *
 * ARGUMENTS:
 *   ecadr  uint16_t  I   the address
 *
 * RETURNS: the word, 15 bits
 **********************************************************************/
static uint16_t word_at(uint16_t ecadr)
{
    uint16_t w = 0; /* the word */

    agc_erase_read_ecadr(ecadr, &w);

    return (w);
}

/* ---- channel output ---------------------------------------------------- */

/**********************************************************************
 * AGC: none -- DNTMEXIT: A to channel 34, L to channel 35.
 *
 * ARGUMENTS:
 *   a  uint16_t  I   word to channel 34 (DNTM1)
 *   l  uint16_t  I   word to channel 35 (DNTM2)
 **********************************************************************/
static void dntmexit(uint16_t a, uint16_t l)
{
    agc_chan_write((int) AGC_CHAN_DNTM1, (uint16_t) (a & WORD15));
    agc_chan_write((int) AGC_CHAN_DNTM2, (uint16_t) (l & WORD15));
}

/**********************************************************************
 * AGC: none -- WOZERO: word order code to zero (CHAN13 bit 7).
 **********************************************************************/
static void wozero(void)
{
    agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (WORD15 & ~BIT7));
}

/**********************************************************************
 * AGC: none -- SENDID: word order zero, then the ID word and LOWIDCOD.
 *
 * ARGUMENTS:
 *   id  uint16_t  I   the ID word (complemented DNLSTCOD or ERASID)
 **********************************************************************/
static void sendid(uint16_t id)
{
    wozero();

    dntmexit(id, (uint16_t) AGC_DN_LOWIDCOD);
}

/* ---- the downlist ------------------------------------------------------ */

/**********************************************************************
 * AGC: none -- DNTABLE or a DNPTR's sublist for the vehicle flown
 *
 * ARGUMENTS:
 *   sub  bool             I   true: a sublist, false: a control list
 *                             (DNTABLE)
 *   ref  int              I   sublist id or DNLSTCOD
 *   out  agc_dn_list_t *  O   the list
 *
 * RETURNS: the list table's status (BAD_ARG for an id or code out of
 *          range)
 **********************************************************************/
static agc_status_t list_lookup(bool sub, int ref, agc_dn_list_t *out)
{
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return (sub ? agc_dnlist_lm_sublist(ref, out) : agc_dnlist_lm_control(ref, out)); /* DNPTR / DNTABLE */
    }

    if (sub)
    {
        return (agc_dnlist_csm_sublist(ref, out)); /* DNPTR */
    }

    agc_dnlist_csm_control(ref, out); /* DNTABLE */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- Take the next entry of a list (NEXTINCL / NEXTINSL): advance, and mark
 * the list finished after its last (negative) entry.
 *
 * ARGUMENTS:
 *   c  tm_cursor_t *  I/O the list cursor
 *
 * RETURNS: the entry taken
 **********************************************************************/
static agc_dn_entry_t cursor_take(tm_cursor_t *c)
{
    agc_dn_entry_t e = c->list.entry[c->next]; /* the entry taken */

    c->next++;

    if (c->next >= c->list.count)
    {
        c->active = false;
    }

    return (e);
}

/**********************************************************************
 * AGC: none -- FETCH2WD: send the next two SP words of the nDNADR in progress.
 **********************************************************************/
static void fetch2wd(void)
{
    uint16_t a = word_at(dn_ecadr);                   /* word to channel 34 */
    uint16_t l = word_at((uint16_t) (dn_ecadr + 1u)); /* word to channel 35 */

    dn_ecadr = (uint16_t) (dn_ecadr + 2u); /* ADS DNECADR (74001)      */
    dn_left--;

    dntmexit(a, l);
}

/**********************************************************************
 * AGC: none -- SNAPLOOP / SNAPEND: copy all but the last entry of a
 * snapshot sublist into DNTMBUFF (DCA, DXCH DNTMBUFF +TMINDEX) and send
 * the last one now.
 *
 * ARGUMENTS:
 *   sub  const agc_dn_list_t *  I   the snapshot sublist (1DNADRs that fit
 *                                   DNTMBUFF: tests/io/service_verbs_telemetry.c)
 *
 * RETURNS: AGC_STATUS_OK, or the status of a store into DNTMBUFF
 **********************************************************************/
static agc_status_t snapshot(const agc_dn_list_t *sub)
{
    agc_status_t   st  = AGC_STATUS_OK;              /* store status */
    agc_dn_entry_t now = sub->entry[sub->count - 1]; /* the entry sent now */

    for (int i = 0; i + 1 < sub->count && st == AGC_STATUS_OK; i++)
    {
        uint16_t to = (uint16_t) (ADR_DNTMBUFF + 2u * (uint16_t) i); /* DNTMBUFF +TMINDEX */

        st = agc_erase_write_ecadr(to, word_at(sub->entry[i].ref));

        if (st == AGC_STATUS_OK)
        {
            st = agc_erase_write_ecadr((uint16_t) (to + 1u), word_at((uint16_t) (sub->entry[i].ref + 1u)));
        }
    }

    sublist.active = false; /* SUBLIST, DNECADR = -1    */
    dn_left        = 0;

    dntmexit(word_at(now.ref), word_at((uint16_t) (now.ref + 1u)));

    return (st);
}

/**********************************************************************
 * AGC: none -- SETWO .. DODNCHAN: act on one list entry.  from_control: the TIME2 test
 * (word order code to zero) is made only for control-list entries.  The
 * list tables obey the AGC's own constraints (no DNCHAN 0, sublists do
 * not nest, snapshot sublists hold 1DNADRs that fit DNTMBUFF), checked
 * over every list in tests/io/service_verbs_telemetry.c.
 *
 * ARGUMENTS:
 *   e             agc_dn_entry_t  I   the list entry
 *   from_control  bool            I   true for a control-list entry
 *                                     (TIME2 test)
 *
 * RETURNS: AGC_STATUS_OK, or the sublist lookup status
 **********************************************************************/
static agc_status_t do_entry(agc_dn_entry_t e, bool from_control)
{
    agc_dn_entry_t now = e; /* the entry sent now (a sublist's first) */

    if (from_control && e.op == 1 && e.ref == ADR_TIME2)
    {
        wozero(); /* SETWO                    */
    }

    if (e.op == AGC_DN_OP_PTR)
    {
        /* DODNPTR: the entry names a sublist.                                     */
        agc_dn_list_t sub;                                       /* the DNPTR's sublist */
        agc_status_t  st = list_lookup(true, (int) e.ref, &sub); /* lookup status */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        if (sub.snapshot)
        {
            return (snapshot(&sub)); /* SNAPLOOP / SNAPEND */
        }

        /* DOSUBLST, NEXTINSL: the sublist's first entry, then the common
         * coding at SETWO +2.                                                 */
        sublist.list   = sub;
        sublist.next   = 0;
        sublist.active = true;
        now            = cursor_take(&sublist);
    }

    if (now.op == AGC_DN_OP_CHAN)
    {
        /* DODNCHAN: channels n, n+1.                                              */
        dntmexit(agc_chan_read((int) now.ref), agc_chan_read((int) now.ref + 1));

        return (AGC_STATUS_OK);
    }

    dn_ecadr = now.ref;
    dn_left  = now.op;

    fetch2wd();

    return (AGC_STATUS_OK);
} /* do_entry */

/**********************************************************************
 * AGC: none -- NEWLIST: start the list DNLSTCOD; the ID word is -DNLSTCOD.
 *
 * RETURNS: AGC_STATUS_OK, or the list lookup status (DNLSTCOD out of
 *          range)
 **********************************************************************/
static agc_status_t newlist(void)
{
    agc_status_t st = list_lookup(false, dnlstcod, &ctlist.list); /* lookup status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    ctlist.next   = 0;
    ctlist.active = true;

    sendid((uint16_t) (WORD15 - (uint16_t) dnlstcod)); /* CS DNLSTCOD */

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- DNPHASE1: reset the pointers and restart the current list.
 *
 * RETURNS: the NEWLIST status
 **********************************************************************/
static agc_status_t dnphase1(void)
{
    sublist.active = false;
    dn_left        = 0;
    dntmgoto       = TM_PHASE2;

    return (newlist());
}

/* ---- erasable dump (V74) ----------------------------------------------- */

/**********************************************************************
 * AGC: none -- DNDUMPI +2 (SENDID with the dump ID).
 **********************************************************************/
static void dndump_id(void)
{
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM); /* LM build: ERASID 1776 */

    dntmgoto = TM_DUMP_LOC;
    wozero();

    dntmexit(lm ? ERASID_LM : ERASID_CSM, (uint16_t) AGC_DN_LOWIDCOD); /* ERASID */
}

/**********************************************************************
 * AGC: none -- DNDUMP2: EBANK and relative address from DUMPLOC, send the register
 * pair (high word = E,1400 +R to channel 34, the next one to 35).  The
 * AGC MASKs with -0 so editing registers 20-23 are not altered.
 *
 * RETURNS: none
 **********************************************************************/
static void dndump2(void)
{
    uint16_t ecadr = (uint16_t) (dumploc & ECADR_MASK); /* ECADR of the high register */
    uint16_t high  = 0;                                 /* E,1400 +R */
    uint16_t low   = 0;                                 /* E,1401 +R */

    agc_erase_read_ecadr(ecadr, &high);
    agc_erase_read_ecadr((uint16_t) (ecadr + 1u), &low);
    dntmexit(high, low);
}

/* ---- public ------------------------------------------------------------ */

/**********************************************************************
 * AGC: none -- DODOWNTM (DOWNRUPT, every 20 ms): word order code to 1 (LM
 * WOTEST: only when it is 0), then the phase in DNTMGOTO.
 *
 * RETURNS: AGC_STATUS_OK, or the list lookup / image status; BAD_ARG for
 *          a bad DNTMGOTO
 **********************************************************************/
agc_status_t agc_telem_downrupt(void)
{
    /* Comanche (C p.1096): CA BIT7, EXTEND, WOR CHAN13 on every DOWNRUPT.
     * Luminary WOTEST / WO1 (L p.997): only when the bit is 0. A channel
     * write is a packet to every peripheral, so the difference shows.  */
    bool lm = agc_state_vehicle() == AGC_VEHICLE_LM; /* Luminary WOTEST */

    if (!lm || (agc_chan_read((int) AGC_CHAN_CHAN13) & BIT7) == 0u)
    {
        agc_chan_or((int) AGC_CHAN_CHAN13, (uint16_t) BIT7); /* WO1: EXTEND, WOR CHAN13 */
    }

    switch (dntmgoto)
    {
        case TM_PHASE1:
            return (dnphase1());
        case TM_PHASE2:
            /* DNPHASE2: the next word of the downlist.                                */
        {
            if (dn_left > 0)
            { /* CCS DNECADR: in progress */
                fetch2wd();

                return (AGC_STATUS_OK);
            }

            if (sublist.active)
            { /* NEXTINSL                 */
                agc_dn_entry_t e = cursor_take(&sublist); /* NEXTINSL / NEXTINCL entry */

                return (do_entry(e, false));
            }

            if (!ctlist.active)
            { /* CHKLIST: end of list     */
                return (newlist());
            }

            agc_dn_entry_t e = cursor_take(&ctlist); /* NEXTINCL                 */

            return (do_entry(e, true));
        }
        case TM_DUMP_ID:
            dndump_id();

            return (AGC_STATUS_OK);
        case TM_DUMP_LOC: /* DUMPLOC and TIME1        */
            dntmgoto = TM_DUMP_FIRST;

            dntmexit(dumploc, word_at(ADR_TIME1)); /* CA TIME1 */

            return (AGC_STATUS_OK);
        case TM_DUMP_FIRST: /* DNDUMP1 -> DNDUMP2       */
            dntmgoto = TM_DUMP_NEXT;

            dndump2();

            return (AGC_STATUS_OK);
        case TM_DUMP_NEXT:
            /* DNDUMP: step DUMPLOC by one DP word; at the end of an EBANK either end
             * the dump (DUMPCNT complete dumps) or send the next EBANK's ID.         */
        {
            dumploc = (uint16_t) ((dumploc + 2u) & WORD15);

            if ((dumploc & LOW8) != 0u)
            {
                dndump2();

                return (AGC_STATUS_OK);
            }

            if ((dumploc & dumpcnt & PRIO34) != 0u)
            {
                return (dnphase1()); /* resume the downlist      */
            }

            dndump_id();

            return (AGC_STATUS_OK);
        }
        default:
            break;
    } /* switch (dntmgoto) */

    return (AGC_STATUS_BAD_ARG);
} /* agc_telem_downrupt */

/**********************************************************************
 * DOFSTART (FRESH_START_AND_RESTART C p.184 / L p.212): TS DNLSTCOD
 * with +0, the P00 downlist; the LM also zeroes AGSWORD (allow AGS
 * initialization). DNTMGOTO is STARTSUB's (agc_telem_restart).
 **********************************************************************/
void agc_telem_fresh_start(void)
{
    agc_telem_select_list(0);

    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_erase_set_agsword(0);
    }
}

/**********************************************************************
 * SLAP1 (L p.211): CA BIT12, TS DUMPCNT -- initialize the downlink
 * erasable memory dump for one pass (LM).
 **********************************************************************/
void agc_telem_slap1(void)
{
    dumpcnt = 004000u; /* BIT12 */
}

/**********************************************************************
 * STARTSUB (FRESH_START_AND_RESTART C p.190 / L p.219): CAF LDNPHAS1, TS
 * DNTMGOTO -- the next DOWNRUPT restarts the current downlist.
 **********************************************************************/
void agc_telem_restart(void)
{
    dntmgoto = TM_PHASE1;
    dn_left  = 0;
}

/**********************************************************************
 * AGC: none -- TS DNLSTCOD: select the list sent from the next NEWLIST on.
 *
 * ARGUMENTS:
 *   code  int  I   DNLSTCOD, a list code (0..4 CM, 0..5 LM); a store by
 *                  address (image load, V21) may hold any word
 **********************************************************************/
void agc_telem_select_list(int code)
{
    dnlstcod = code; /* a list code, AGSWORD, or a word stored at its address (NEWLIST refuses one outside DNTABLE) */
}

/**********************************************************************
 * AGC: none -- DNLSTCOD, read: the code of the list sent from the next
 * NEWLIST on (the erasable map's address 0332, both ropes)
 *
 * RETURNS: DNLSTCOD
 **********************************************************************/
uint16_t agc_erase_dnlstcod(void)
{
    return ((uint16_t) dnlstcod);
}

/**********************************************************************
 * AGC: none -- DUMPCNT, read (the erasable map's 0333, both ropes)
 *
 * RETURNS: DUMPCNT (BIT12 one dump, BIT13 two, BIT14 four; the low
 *          bits count the dumps sent)
 **********************************************************************/
uint16_t agc_erase_dumpcnt(void)
{
    return (dumpcnt);
}

/**********************************************************************
 * AGC: none -- DUMPCNT, written by address (an image load, V21)
 *
 * ARGUMENTS:
 *   v  uint16_t  I   DUMPCNT
 **********************************************************************/
void agc_erase_set_dumpcnt(uint16_t v)
{
    dumpcnt = v;
}

/**********************************************************************
 * AGC: none -- V74 DNEDUMP: DNTMGOTO = DNDUMPI (DUMPLOC = 0).
 **********************************************************************/
void agc_telem_start_dump(void)
{
    dumploc  = 0;
    dntmgoto = TM_DUMP_ID;
}

/**********************************************************************
 * AGC: none -- AGSWORD (LM): the downlist to return to after AGS initialization.
 *
 * RETURNS: AGSWORD
 **********************************************************************/
uint16_t agc_erase_agsword(void)
{
    return (agsword);
}

/**********************************************************************
 * AGC: none -- AGSWORD (LM), written: TS AGSWORD in SEUDOP00 and P00
 *
 * ARGUMENTS:
 *   v  uint16_t  I   the list code to return to after AGS initialization
 **********************************************************************/
void agc_erase_set_agsword(uint16_t v)
{
    agsword = v; /* 0 or a list code from the SEUDOP00 table */
}

/**********************************************************************
 * AGC: none -- DP words one DNPTR sends at its place in the list (*words): a snapshot
 * sublist sends only its last entry now (the rest go out via DNTMBUFF
 * entries). Returns the status of the sublist lookup.
 *
 * ARGUMENTS:
 *   ref    int    I   sublist id
 *   words  int *  O   DP words the DNPTR sends now
 *
 * RETURNS: AGC_STATUS_OK or the sublist lookup status
 **********************************************************************/
static agc_status_t sublist_words(int ref, int *words)
{
    agc_dn_list_t sub;                               /* the DNPTR's sublist */
    agc_status_t  st = list_lookup(true, ref, &sub); /* lookup status */

    *words = 0;

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (sub.snapshot)
    {
        *words = 1;
    }
    else
    {
        for (int i = 0; i < sub.count; i++)
        {
            uint8_t op = sub.entry[i].op; /* entry op code */

            *words += (op == AGC_DN_OP_CHAN) ? 1 : (int) op; /* DNCHAN: one DP word */
        }
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none -- Number of DP words a downlist sends, the ID word included.
 *
 * ARGUMENTS:
 *   code   int            I   DNLSTCOD
 *   words  int *          O   DP words of the downlist, ID included
 *
 * RETURNS: AGC_STATUS_OK, or the lookup status
 **********************************************************************/
agc_status_t agc_dnlist_word_count(int code, int *words)
{
    agc_dn_list_t list;                                 /* the control list */
    agc_status_t  st = list_lookup(false, code, &list); /* lookup status */

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    int n = 1; /* ID / sync word           */

    for (int i = 0; i < list.count && st == AGC_STATUS_OK; i++)
    {
        agc_dn_entry_t e = list.entry[i];                             /* the entry */
        int            k = (e.op == AGC_DN_OP_CHAN) ? 1 : (int) e.op; /* DP words of the entry */

        if (e.op == AGC_DN_OP_PTR)
        {
            st = sublist_words((int) e.ref, &k);
        }

        n += k;
    }

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    *words = n;

    return (AGC_STATUS_OK);
}
