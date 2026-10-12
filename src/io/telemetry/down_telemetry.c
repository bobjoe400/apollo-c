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
 *   (AGC erasables reached through accessors)
 *   Name          Type      I/O  Scale  Description
 *   ------------  --------  ---  -----  ----------------------------------
 *   the downlist  various   I    as     every item the lists name
 *   items                         kept
 *   CHAN13        uint16_t  I/O  bits   bit 7: word order code
 *   DUMPLOC words uint16_t  I    words  the dumped registers (ECADR image)
 *
 * EXTERNAL REFERENCES:
 *   Name                   Description
 *   ---------------------  ------------------------------------------
 *   agc_dnlist_*           the list tables (downlink_lists_{csm,lm}.c)
 *   agc_erase_read_ecadr   DNDUMP2's register pair
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
 *   DEVIATION (data words): the AGC reads the erasable addressed by the
 *   list. The C build has no flat erasable memory, so each downlist item
 *   is read through the owner's accessor where one exists (see item_word
 *   below); items whose owner exports no accessor yet are sent as zero
 *   (ERASZERO), which the AGC itself sends for SPARE entries.  The
 *   erasable dump reads its words by address (agc_erase_read_ecadr), so
 *   it sends the flagwords and whatever the address image holds; typed
 *   erasables are not mapped there yet (src/core/ecadr.c DEVIATION).
 *
 *   DUMPCNT is set by SLAP1 (LM fresh start, one dump).  The V21N01 load
 *   of DUMPCNT the AGC allows goes to the address image and does not
 *   reach it (same DEVIATION); with DUMPCNT = 0 the AGC, and this code,
 *   dump forever.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "agc/lm/dap.h"
#include "agc/programs/rendezvous.h"

#define WORD15         077777u /* one AGC word */
#define BIT7           000100u /* CHAN13 bit 7: downlink word order code   */
#define LOW8           000377u /* relative address in an EBANK */
#define PRIO34         034000u /* DUMPLOC dump counter, bits 12-14         */
#define ERASID_CSM     001777u /* ERASID EQUALS LOW10 (Comanche)           */
#define ERASID_LM      001776u /* ERASID EQUALS BITS2-10 (Luminary)        */
#define DNTMBUFF_WORDS 22      /* DNTMBUFF ERASE +21D: 11 DP words         */
#define ECADR_MASK     003777u /* DUMPLOC EEE RRRRRRRR: the ECADR dumped   */

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

static tm_phase_t    dntmgoto = TM_PHASE1;     /* DNTMGOTO, the next phase */
static int           dnlstcod;                 /* DNLSTCOD, B(1)PRM list code      */
static uint16_t      agsword;                  /* AGSWORD (LM): saved DNLSTCOD     */
static uint16_t      dumpcnt;                  /* DUMPCNT, B(1)PRM                 */
static uint16_t      dumploc;                  /* DUMPLOC: dump counter + ECADR    */
static tm_cursor_t   ctlist;                   /* CTLIST                           */
static tm_cursor_t   sublist;                  /* SUBLIST                          */
static agc_dn_item_t dn_item;                  /* DNECADR: item of the nDNADR      */
static int           dn_offset;                /*   its next SP-word offset        */
static int           dn_left;                  /*   DP words still to send (> 0)   */
static uint16_t      dntmbuff[DNTMBUFF_WORDS]; /* DNTMBUFF snapshot buffer    */

/* ---- word images ------------------------------------------------------- */

/**********************************************************************
 * AGC: none -- a signed SP value as the 15-bit ones' complement word the AGC sends
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I   signed SP value
 *
 * RETURNS: the 15-bit ones' complement word
 **********************************************************************/
static uint16_t sp_word(agc_sp_t v)
{
    int32_t w = (v >= 0) ? (int32_t) v : (int32_t) WORD15 + (int32_t) v; /* the word as an int */

    return ((uint16_t) w);
}

/**********************************************************************
 * AGC: none -- Word k (0 = high, 1 = low) of a DP value.
 *
 * ARGUMENTS:
 *   v  agc_dp_t  I   DP value
 *   k  int       I   0 high word, 1 low word
 *
 * RETURNS: that word, 15 bits
 **********************************************************************/
static uint16_t dp_word(agc_dp_t v, int k)
{
    return (sp_word((k == 0) ? agc_dp_high(v) : agc_dp_low(v))); /* major / minor word */
}

/**********************************************************************
 * AGC: none -- Word k (0..5) of a DP vector, X high first.
 *
 * ARGUMENTS:
 *   v  agc_vec_t  I   DP vector
 *   k  int        I   word 0..5 (X high first)
 *
 * RETURNS: that word, 15 bits
 **********************************************************************/
static uint16_t vec_word(agc_vec_t v, int k)
{
    return (dp_word(v.c[k / 2], k % 2));
}

/**********************************************************************
 * AGC: none -- CDU and PIPA counter block, registers 032-042 in order: CDUX CDUY CDUZ
 * CDUT CDUS PIPAX PIPAY PIPAZ.
 *
 * ARGUMENTS:
 *   k  int  I   0..7: CDUX CDUY CDUZ CDUT CDUS PIPAX PIPAY PIPAZ
 *
 * RETURNS: the counter word (0 past the block)
 **********************************************************************/
static uint16_t counter_word(int k)
{
    switch (k)
    {
        case 0:
            return (agc_erase_cdux());
        case 1:
            return (agc_erase_cduy());
        case 2:
            return (agc_erase_cduz());
        case 3:
            return (agc_erase_cdut());
        case 4:
            return (agc_erase_cdus());
        case 5:
            return (sp_word(agc_erase_pipax()));
        case 6:
            return (sp_word(agc_erase_pipay()));
        case 7:
            return (sp_word(agc_erase_pipaz()));
        default:
            break;
    }

    return (0);
}

/**********************************************************************
 * AGC: none -- word k of a small block of consecutive registers (the
 * downlist sends them as they lie in erasable memory)
 *
 * ARGUMENTS:
 *   k     int               I   SP-word offset in the item
 *   n     int               I   words in the block
 *   list  const uint16_t *  I   the block's words
 *
 * RETURNS: list[k], or 0 past the block
 **********************************************************************/
static uint16_t pick(int k, int n, const uint16_t *list)
{
    uint16_t w = 0u; /* past the block: 0 */

    if (k >= 0 && k < n)
    {
        w = list[k];
    }

    return (w);
}

/**********************************************************************
 * AGC: none -- Items that are plain DP times or vectors from agc_state.h.
 *
 * ARGUMENTS:
 *   item  agc_dn_item_t  I   the downlist item
 *   k     int            I   SP-word offset in the item
 *   w     uint16_t *     O   the word
 *
 * RETURNS: true when the item is one of these
 **********************************************************************/
static bool item_word_state(agc_dn_item_t item, int k, uint16_t *w)
{
    bool      vec = true; /* the item is a vector */
    agc_vec_t v   = {{0, 0, 0}};
    agc_dp_t  t   = 0;    /* the DP time, cs 2^28 */

    switch ((int) item)
    {
        case AGC_DN_RN:
            v   = agc_erase_rn();
            break;
        case AGC_DN_VN:
            v   = agc_erase_vn();
            break;
        case AGC_DN_DELV:
            v   = agc_erase_delv();
            break;
        case AGC_DN_VGTIG:
            v   = agc_erase_vgtig();
            break;
        case AGC_DN_DELVSLV:
            v   = agc_rendz_erasable()->delvlvc; /* DELVSLV = DELVLVC */
            break;
        case AGC_DN_RLS:
            v   = agc_erase_rls();
            break;
        case AGC_DN_PIPTIME:
            vec = false;
            t   = agc_erase_piptime();
            break;
        case AGC_DN_TIG:
            vec = false;
            t   = agc_erase_tig();
            break;
        case AGC_DN_TGO:
            vec = false;
            t   = agc_erase_tgo();
            break;
        case AGC_DN_TEVENT:
            vec = false;
            t   = agc_erase_tevent();
            break;
        case AGC_DN_TTPI:
            vec = false;
            t   = agc_erase_ttpi();
            break;
        default:
            return (false);
    } /* switch ((int) item) */

    if (vec)
    {
        *w = (k < 2 * AGC_VEC_N) ? vec_word(v, k) : 0u; /* 0 past the vector */
    }
    else
    {
        *w = (k < 2) ? dp_word(t, k) : 0u; /* 0 past the DP */
    }

    return (true);
} /* item_word_state */

/**********************************************************************
 * AGC: none -- small multi-erasable items: the AGC sends consecutive registers
 *
 * ARGUMENTS:
 *   item  agc_dn_item_t  I   the downlist item
 *   k     int            I   SP-word offset in the item
 *   w     uint16_t *     O   the word
 *
 * RETURNS: true when the item is one of these
 **********************************************************************/
static bool item_word_block(agc_dn_item_t item, int k, uint16_t *w)
{
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM); /* LM build */

    switch ((int) item)
    {
        case AGC_DN_TIME2:
        {
            const uint16_t r[2] = {sp_word(agc_erase_time2()), sp_word(agc_erase_time1())}; /* TIME2, TIME1 */

            *w = pick(k, 2, r);
        }
        break;
        case AGC_DN_STATE:
        {
            int nflag = lm ? AGC_FLAGWORDS_LM : AGC_FLAGWORDS_CSM; /* flagwords of the rope */

            *w = (k < nflag) ? agc_flagword_get(k) : 0u; /* FLAGWRD0 .. */
        }
        break;
        case AGC_DN_DSPTAB:
            *w = (k < 12) ? agc_erase_dsptab(k) : 0u; /* DSPTAB +0..+11D */
            break;
        case AGC_DN_CDUX:
            *w = counter_word(k);
            break;
        case AGC_DN_CDUZ:
            *w = counter_word(2 + k);
            break;
        case AGC_DN_CDUS:
            *w = counter_word(4 + k);
            break;
        case AGC_DN_IMODES30:
        {
            const uint16_t r[2] = {agc_erase_imodes30(), agc_erase_imodes33()}; /* IMODES30, IMODES33 */

            *w = pick(k, 2, r);
        }
        break;
        case AGC_DN_OPTMODES:
            *w = (k == 0 && !lm) ? agc_erase_optmodes() : 0u;   /* OPTMODES (CM); HOLDFLAG not sent */
            break;
        case AGC_DN_RADMODES:
            /* RADMODES,DAPBOOLS = FLGWRD12,FLGWRD13 (LM only) */
            *w = (lm && k < 2) ? agc_flagword_get(12 + k) : 0u; /* FLGWRD12, 13 */
            break;
        case AGC_DN_LEMMASS:
        {
            const uint16_t r[2] = {sp_word(agc_erase_lemmass()), sp_word(agc_erase_csmmass())}; /* LEMMASS, CSMMASS */

            *w = pick(k, 2, r);
        }
        break;
        case AGC_DN_DAPDATR1:
        {
            const uint16_t r[2] = {agc_erase_dapdatr1(), agc_erase_dapdatr2()}; /* DAPDATR1, DAPDATR2 */

            *w = pick(k, 2, r);
        }
        break;
        case AGC_DN_CADRFLSH:
            *w = (k >= 3 && k < 6) ? agc_erase_failreg(k - 3) : 0u; /* FAILREG +0..+2 after CADRFLSH */
            break;
        case AGC_DN_REDOCTR:
        {
            const uint16_t r[4] = {agc_restart_redoctr(), agc_erase_thetad(0), agc_erase_thetad(1), agc_erase_thetad(2)}; /* REDOCTR, THETAD +0..+2 */

            *w = pick(k, 4, r);
        }
        break;
        case AGC_DN_COMPNUMB:
            *w = agc_uplink_update_dnword(k);
            break;
        case AGC_DN_UPBUFF:
            *w = agc_uplink_update_dnword(4 + k);
            break;
        case AGC_DN_DNTMBUFF:
            *w = (k < DNTMBUFF_WORDS) ? dntmbuff[k] : 0u; /* the snapshot buffer */
            break;
        default:
            return (false);
    } /* switch ((int) item) */

    return (true);
} /* item_word_block */

/**********************************************************************
 * AGC: none -- LM DAP downlink words (agc_lm_dap.h).
 *
 * ARGUMENTS:
 *   item  agc_dn_item_t  I   the downlist item
 *   k     int            I   SP-word offset in the item
 *   w     uint16_t *     O   the word
 *
 * RETURNS: true when the item is one of these (LM only)
 **********************************************************************/
static bool item_word_lm_dap(agc_dn_item_t item, int k, uint16_t *w)
{
    if (agc_state_vehicle() != AGC_VEHICLE_LM)
    {
        return (false);
    }

    switch ((int) item)
    {
        case AGC_DN_CDUXD:
        {
            const uint16_t r[3] = {agc_erase_cduxd(), agc_erase_cduyd(), agc_erase_cduzd()}; /* CDUXD, CDUYD, CDUZD */

            *w = pick(k, 3, r);
        }
        break;
        case AGC_DN_OMEGAP:
        {
            const uint16_t r[3] = {sp_word(agc_erase_omegap()), sp_word(agc_erase_omegaq()), sp_word(agc_erase_omegar())}; /* OMEGAP, OMEGAQ, OMEGAR */

            *w = pick(k, 3, r);
        }
        break;
        case AGC_DN_OMEGAPD:
        {
            const uint16_t r[3] = {sp_word(agc_erase_omegapd()), sp_word(agc_erase_omegaqd()), sp_word(agc_erase_omegard())}; /* OMEGAPD, OMEGAQD, OMEGARD */

            *w = pick(k, 3, r);
        }
        break;
        case AGC_DN_ALPHAQ:
        {
            const uint16_t r[2] = {sp_word(agc_erase_alphaq()), sp_word(agc_erase_alphar())}; /* ALPHAQ, ALPHAR */

            *w = pick(k, 2, r);
        }
        break;
        default:
            return (false);
    }

    return (true);
} /* item_word_lm_dap */

/**********************************************************************
 * AGC: none -- SP word at offset k of a downlist item (FETCH2WD's DCA 1400 +ECADR).
 *
 * ARGUMENTS:
 *   item  agc_dn_item_t  I   the downlist item (DNADR ECADR)
 *   k     int            I   SP-word offset in the item
 *
 * RETURNS: the word sent, 15 bits (0 for SPARE and items without an
 *          accessor)
 **********************************************************************/
static uint16_t item_word(agc_dn_item_t item, int k)
{
    uint16_t w = 0; /* the word (accessor items) */

    if (item == AGC_DN_R_OTHER || item == AGC_DN_V_OTHER || item == AGC_DN_T_OTHER)
    {
        /* The other vehicle's state vector words (R-OTHER, V-OTHER, T-OTHER).     */
        agc_state_vector_t sv; /* the other vehicle's state vector */

        agc_sv_get(AGC_SV_OTHER, &sv);

        if (item == AGC_DN_R_OTHER)
        {
            return (vec_word(sv.r, k));
        }

        if (item == AGC_DN_V_OTHER)
        {
            return (vec_word(sv.v, k));
        }

        return ((k < 2) ? dp_word(sv.t, k) : 0u); /* T-OTHER */
    }

    if (item == AGC_DN_REFSMMAT)
    {
        /* REFSMMAT: 9 DP elements row by row = 18 words.                          */
        if (k >= 6 * AGC_VEC_N)
        {
            return (0); /* past the matrix */
        }

        agc_mat_t m = agc_erase_refsmmat(); /* REFSMMAT, half-unit */

        return (vec_word(m.row[k / 6], k % 6));
    }

    if (item_word_state(item, k, &w) || item_word_block(item, k, &w) || item_word_lm_dap(item, k, &w))
    {
        return ((uint16_t) (w & WORD15));
    }

    return (0); /* ERASZERO / no accessor   */
} /* item_word */

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

    return (sub ? agc_dnlist_csm_sublist(ref, out) : agc_dnlist_csm_control(ref, out)); /* DNPTR / DNTABLE */
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
    uint16_t a = item_word(dn_item, dn_offset);     /* word to channel 34 */
    uint16_t l = item_word(dn_item, dn_offset + 1); /* word to channel 35 */

    dn_offset += 2; /* ADS DNECADR (74001)      */
    dn_left--;

    dntmexit(a, l);
}

/**********************************************************************
 * AGC: none -- SETWO .. DODNCHAN: act on one list entry.  from_control: the TIME2 test
 * (word order code to zero) is made only for control-list entries.  The
 * list tables obey the AGC's own constraints (no DNCHAN 0, sublists do
 * not nest, snapshot sublists hold 1DNADRs that fit DNTMBUFF), checked
 * over every list in tests/io/sysmisc.c.
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

    if (from_control && e.op == 1 && e.ref == (uint16_t) AGC_DN_TIME2)
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
            /* SNAPLOOP / SNAPEND: copy all but the last entry of the snapshot
             * into DNTMBUFF and send the last one now.                             */
            for (int i = 0; i + 1 < sub.count; i++)
            {
                dntmbuff[2 * i]     = item_word((agc_dn_item_t) sub.entry[i].ref, sub.entry[i].offset);
                dntmbuff[2 * i + 1] = item_word((agc_dn_item_t) sub.entry[i].ref, sub.entry[i].offset + 1);
            }

            now            = sub.entry[sub.count - 1];
            sublist.active = false; /* SUBLIST, DNECADR = -1    */
            dn_left        = 0;

            dntmexit(item_word((agc_dn_item_t) now.ref, now.offset), item_word((agc_dn_item_t) now.ref, now.offset + 1));

            return (AGC_STATUS_OK);
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

    dn_item   = (agc_dn_item_t) now.ref;
    dn_offset = now.offset;
    dn_left   = now.op;

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
 * AGC: none -- DODOWNTM (DOWNRUPT, every 20 ms): word order code to 1, then the phase
 * in DNTMGOTO.
 *
 * RETURNS: AGC_STATUS_OK, or the list lookup / image status; BAD_ARG for
 *          a bad DNTMGOTO
 **********************************************************************/
agc_status_t agc_telem_downrupt(void)
{
    agc_chan_or((int) AGC_CHAN_CHAN13, (uint16_t) BIT7); /* WOTEST/WO1 */

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

            dntmexit(dumploc, sp_word(agc_erase_time1()));

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
 *   code  int  I   DNLSTCOD, a list code (0..4 CM, 0..5 LM)
 **********************************************************************/
void agc_telem_select_list(int code)
{
    dnlstcod = code; /* a list-code constant at every caller */
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
 *   v      agc_vehicle_t  I   the vehicle flown (asserted)
 *   code   int            I   DNLSTCOD
 *   words  int *          O   DP words of the downlist, ID included
 *
 * RETURNS: AGC_STATUS_OK, BAD_ARG for a NULL words or another vehicle, or
 *          the lookup status
 **********************************************************************/
agc_status_t agc_dnlist_word_count(agc_vehicle_t v, int code, int *words)
{
    if (!AGC_ASSERT(words != NULL) || !AGC_ASSERT(v == agc_state_vehicle()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

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
