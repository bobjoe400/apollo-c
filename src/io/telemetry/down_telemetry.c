/*
 * down_telemetry.c -- DOWN-TELEMETRY_PROGRAM.agc (Comanche055 pp. 1093-1102,
 * Luminary099 DOWN_TELEMETRY_PROGRAM.agc pp. 988-997).
 *
 * Every DOWNRUPT (20 ms) the program writes one DP downlink word: the high
 * SP word to channel 34 (DNTM1) and the low one to channel 35 (DNTM2).  A
 * downlist is 100 such words (2 s): the ID / sync pair, then the words
 * named by the control list and its sublists (src/io/telemetry/downlink_
 * lists_{csm,lm}.c).  Channel 13 bit 7 (word order code) is 0 for the ID
 * word and for TIME2,TIME1 and 1 otherwise.  V74 replaces the downlist by
 * an erasable dump (DNDUMPI/DNDUMP), which ends after DUMPCNT complete
 * dumps and resumes the interrupted downlist at DNPHASE1.
 *
 * The two vehicles differ only in DODOWNTM: Luminary sets the word order
 * bit only when it is not already set (WOTEST); Comanche always WORs it.
 * The effect is the same and is modelled once.
 *
 * DEVIATION (DNTMGOTO): the AGC keeps the address of the next phase in
 * DNTMGOTO (DNPHASE1, DNPHASE2, DNDUMPI +3, DNDUMP1, DNDUMP); here it is an
 * enum.  DNECADR (count and ECADR of the nDNADR in progress), CTLIST and
 * SUBLIST (list pointers, negative at the last entry) become an item id,
 * word offset and word count plus list indices with "last entry" flags.
 *
 * DEVIATION (data words): the AGC reads the erasable addressed by the list.
 * The C build has no flat erasable memory, so each downlist item is read
 * through the owner's accessor where one exists (see item_word below);
 * items whose owner exports no accessor yet are sent as zero (ERASZERO),
 * which the AGC itself sends for SPARE entries.  The erasable dump sends
 * DUMPLOC and TIME1 exactly, and zero for the dumped words.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service_internal.h"
#include "agc/lm/dap.h"

#define WORD15         077777u
#define BIT7           000100u /* CHAN13 bit 7: downlink word order code   */
#define LOW8           000377u
#define PRIO34         034000u    /* DUMPLOC dump counter, bits 12-14         */
#define ERASID_CSM     001777u    /* ERASID EQUALS LOW10 (Comanche)           */
#define ERASID_LM      001776u    /* ERASID EQUALS BITS2-10 (Luminary)        */
#define DNTMBUFF_WORDS 22         /* DNTMBUFF ERASE +21D: 11 DP words         */
#define DUMP_LIMIT     (8 * 0400) /* 8 EBANKs of 256 words                    */

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
    agc_dn_list_t list;
    int           next;
    bool          active; /* SUBLIST positive / CTLIST positive        */
} tm_cursor_t;

static tm_phase_t    dntmgoto = TM_PHASE1;
static int           dnlstcod;  /* DNLSTCOD, B(1)PRM list code      */
static uint16_t      agsword;   /* AGSWORD (LM): saved DNLSTCOD     */
static uint16_t      dumpcnt;   /* DUMPCNT, B(1)PRM                 */
static uint16_t      dumploc;   /* DUMPLOC: dump counter + ECADR    */
static tm_cursor_t   ctlist;    /* CTLIST                           */
static tm_cursor_t   sublist;   /* SUBLIST                          */
static agc_dn_item_t dn_item;   /* DNECADR: item of the nDNADR      */
static int           dn_offset; /*   its next SP-word offset        */
static int           dn_left;   /*   DP words still to send (> 0)   */
static uint16_t      dntmbuff[DNTMBUFF_WORDS]; /* DNTMBUFF snapshot buffer    */

/* ---- word images ------------------------------------------------------- */

/* A signed SP value as the 15-bit ones'-complement word the AGC sends.    */
static uint16_t sp_word(agc_sp_t v)
{
    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return 0; /* no valid result */
    }
    int32_t w = (v >= 0) ? (int32_t) v : (int32_t) WORD15 + (int32_t) v;
    if (!AGC_ASSERT(w >= 0 && w <= (int32_t) WORD15))
    {
        return 0; /* no valid result */
    }
    return (uint16_t) w;
}

/* Word k (0 = high, 1 = low) of a DP value.                               */
static uint16_t dp_word(agc_dp_t v, int k)
{
    if (!AGC_ASSERT(k == 0 || k == 1))
    {
        return 0; /* no valid result */
    }
    return sp_word((k == 0) ? agc_dp_high(v) : agc_dp_low(v));
}

/* Word k (0..5) of a DP vector, X high first.                             */
static uint16_t vec_word(agc_vec_t v, int k)
{
    if (!AGC_ASSERT(k >= 0 && k < 2 * AGC_VEC_N))
    {
        return 0;
    }
    return dp_word(v.c[k / 2], k % 2);
}

/* The other vehicle's state vector words (R-OTHER, V-OTHER, T-OTHER).     */
static uint16_t other_word(agc_dn_item_t item, int k)
{
    agc_state_vector_t sv;
    agc_status_t       st = agc_sv_get(AGC_SV_OTHER, &sv);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return 0;
    }
    if (item == AGC_DN_R_OTHER)
    {
        return vec_word(sv.r, k);
    }
    if (item == AGC_DN_V_OTHER)
    {
        return vec_word(sv.v, k);
    }
    return (k < 2) ? dp_word(sv.t, k) : 0u;
}

/* CDU and PIPA counter block, registers 032-042 in order: CDUX CDUY CDUZ
 * CDUT CDUS PIPAX PIPAY PIPAZ.                                            */
static uint16_t counter_word(int k)
{
    if (!AGC_ASSERT(k >= 0))
    {
        return 0; /* no valid result */
    }
    switch (k)
    {
        case 0:
            return agc_erase_cdux();
        case 1:
            return agc_erase_cduy();
        case 2:
            return agc_erase_cduz();
        case 3:
            return agc_erase_cdut();
        case 4:
            return agc_erase_cdus();
        case 5:
            return sp_word(agc_erase_pipax());
        case 6:
            return sp_word(agc_erase_pipay());
        case 7:
            return sp_word(agc_erase_pipaz());
        default:
            break;
    }
    return 0;
}

/* Items that are plain DP times or vectors from agc_state.h.             */
static bool item_word_state(agc_dn_item_t item, int k, uint16_t *w)
{
    if (!AGC_ASSERT(w != NULL && k >= 0))
    {
        return false;
    }
    bool      vec = true;
    agc_vec_t v   = {{0, 0, 0}};
    agc_dp_t  t   = 0;
    switch ((int) item)
    {
        case AGC_DN_RN:
            v = agc_erase_rn();
            break;
        case AGC_DN_VN:
            v = agc_erase_vn();
            break;
        case AGC_DN_DELV:
            v = agc_erase_delv();
            break;
        case AGC_DN_VGTIG:
            v = agc_erase_vgtig();
            break;
        case AGC_DN_DELVSLV:
            v = agc_erase_delvslv();
            break;
        case AGC_DN_RLS:
            v = agc_erase_rls();
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
            return false;
    }
    if (vec)
    {
        *w = (k < 2 * AGC_VEC_N) ? vec_word(v, k) : 0u;
    }
    else
    {
        *w = (k < 2) ? dp_word(t, k) : 0u;
    }
    return true;
}

/* REFSMMAT: 9 DP elements row by row = 18 words.                          */
static uint16_t refsmmat_word(int k)
{
    if (!AGC_ASSERT(k >= 0) || k >= 6 * AGC_VEC_N)
    {
        return 0;
    }
    agc_mat_t m = agc_erase_refsmmat();
    return vec_word(m.row[k / 6], k % 6);
}

/* Small multi-erasable items: the AGC sends consecutive registers.        */
static bool item_word_block(agc_dn_item_t item, int k, uint16_t *w)
{
    if (!AGC_ASSERT(w != NULL && k >= 0))
    {
        return false;
    }
    bool lm = (agc_state_vehicle() == AGC_VEHICLE_LM);
    switch ((int) item)
    {
        case AGC_DN_TIME2:
            *w = (k == 0) ? sp_word(agc_erase_time2()) :
                 (k == 1) ? sp_word(agc_erase_time1()) :
                            0u;
            break;
        case AGC_DN_STATE:
            *w = (k < (lm ? AGC_FLAGWORDS_LM : AGC_FLAGWORDS_CSM)) ?
                     agc_flagword_get(k) :
                     0u;
            break;
        case AGC_DN_DSPTAB:
            *w = (k < 12) ? agc_erase_dsptab(k) : 0u;
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
            *w = (k == 0) ? agc_erase_imodes30() :
                 (k == 1) ? agc_erase_imodes33() :
                            0u;
            break;
        case AGC_DN_OPTMODES:
            *w = (k == 0 && !lm) ? agc_erase_optmodes() : 0u;
            break;
        case AGC_DN_RADMODES:
            *w = (k == 0 && lm) ? agc_erase_radmodes() : 0u;
            break;
        case AGC_DN_LEMMASS:
            *w = (k == 0) ? sp_word(agc_erase_lemmass()) :
                 (k == 1) ? sp_word(agc_erase_csmmass()) :
                            0u;
            break;
        case AGC_DN_DAPDATR1:
            *w = (k == 0) ? agc_erase_dapdatr1() :
                 (k == 1) ? agc_erase_dapdatr2() :
                            0u;
            break;
        case AGC_DN_CADRFLSH:
            *w = (k >= 3 && k < 6) ? agc_erase_failreg(k - 3) : 0u;
            break;
        case AGC_DN_REDOCTR:
            *w = (k == 0) ? agc_restart_redoctr() :
                 (k < 4)  ? agc_erase_thetad(k - 1) :
                            0u;
            break;
        case AGC_DN_COMPNUMB:
            *w = agc_uplink_update_dnword(k);
            break;
        case AGC_DN_UPBUFF:
            *w = agc_uplink_update_dnword(4 + k);
            break;
        case AGC_DN_DNTMBUFF:
            *w = (k < DNTMBUFF_WORDS) ? dntmbuff[k] : 0u;
            break;
        default:
            return false;
    }
    return true;
}

/* LM DAP downlink words (agc_lm_dap.h).                                   */
static bool item_word_lm_dap(agc_dn_item_t item, int k, uint16_t *w)
{
    if (!AGC_ASSERT(w != NULL && k >= 0))
    {
        return false;
    }
    if (agc_state_vehicle() != AGC_VEHICLE_LM)
    {
        return false;
    }
    switch ((int) item)
    {
        case AGC_DN_CDUXD:
            *w = (k == 0) ? agc_erase_cduxd() :
                 (k == 1) ? agc_erase_cduyd() :
                 (k == 2) ? agc_erase_cduzd() :
                            0u;
            break;
        case AGC_DN_OMEGAP:
            *w = (k == 0) ? sp_word(agc_erase_omegap()) :
                 (k == 1) ? sp_word(agc_erase_omegaq()) :
                 (k == 2) ? sp_word(agc_erase_omegar()) :
                            0u;
            break;
        case AGC_DN_OMEGAPD:
            *w = (k == 0) ? sp_word(agc_erase_omegapd()) :
                 (k == 1) ? sp_word(agc_erase_omegaqd()) :
                 (k == 2) ? sp_word(agc_erase_omegard()) :
                            0u;
            break;
        case AGC_DN_ALPHAQ:
            *w = (k == 0) ? sp_word(agc_erase_alphaq()) :
                 (k == 1) ? sp_word(agc_erase_alphar()) :
                            0u;
            break;
        default:
            return false;
    }
    return true;
}

/* SP word at offset k of a downlist item (FETCH2WD's DCA 1400 +ECADR).    */
static uint16_t item_word(agc_dn_item_t item, int k)
{
    if (!AGC_ASSERT(item < AGC_DN_ITEM_COUNT) || !AGC_ASSERT(k >= 0 && k < 64))
    {
        return 0;
    }
    uint16_t w = 0;
    if (item == AGC_DN_R_OTHER || item == AGC_DN_V_OTHER ||
        item == AGC_DN_T_OTHER)
    {
        return other_word(item, k);
    }
    if (item == AGC_DN_REFSMMAT)
    {
        return refsmmat_word(k);
    }
    if (item_word_state(item, k, &w) || item_word_block(item, k, &w) ||
        item_word_lm_dap(item, k, &w))
    {
        return (uint16_t) (w & WORD15);
    }
    return 0; /* ERASZERO / no accessor   */
}

/* ---- channel output ---------------------------------------------------- */

/* DNTMEXIT: A to channel 34, L to channel 35.                             */
static agc_status_t dntmexit(uint16_t a, uint16_t l)
{
    agc_chan_write((int) AGC_CHAN_DNTM1, (uint16_t) (a & WORD15));
    agc_chan_write((int) AGC_CHAN_DNTM2, (uint16_t) (l & WORD15));
    return AGC_STATUS_OK;
}

/* WOZERO: word order code to zero (CHAN13 bit 7).                         */
static void wozero(void)
{
    agc_chan_and((int) AGC_CHAN_CHAN13, (uint16_t) (WORD15 & ~BIT7));
    if (!AGC_ASSERT((agc_chan_read((int) AGC_CHAN_CHAN13) & BIT7) == 0u))
    {
        return;
    }
}

/* SENDID: word order zero, then the ID word and LOWIDCOD.                 */
static agc_status_t sendid(uint16_t id)
{
    if (!AGC_ASSERT(id <= WORD15))
    {
        return AGC_STATUS_BAD_ARG;
    }
    wozero();
    return dntmexit(id, (uint16_t) AGC_DN_LOWIDCOD);
}

/* ---- the downlist ------------------------------------------------------ */

static agc_status_t list_lookup(bool sub, int ref, agc_dn_list_t *out)
{
    if (!AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        return sub ? agc_dnlist_lm_sublist(ref, out) :
                     agc_dnlist_lm_control(ref, out);
    }
    return sub ? agc_dnlist_csm_sublist(ref, out) :
                 agc_dnlist_csm_control(ref, out);
}

/* Take the next entry of a list (NEXTINCL / NEXTINSL): advance, and mark
 * the list finished after its last (negative) entry.                       */
static agc_dn_entry_t cursor_take(tm_cursor_t *c)
{
    agc_dn_entry_t e = {1, true, (uint16_t) AGC_DN_SPARE, 0};
    if (!AGC_ASSERT(c != NULL && c->active && c->next < c->list.count))
    {
        return e;
    }
    e = c->list.entry[c->next];
    c->next++;
    if (c->next >= c->list.count)
    {
        if (!AGC_ASSERT(e.neg)) /* -xDNADR ends every list   */
        {
            return e;
        }
        c->active = false;
    }
    return e;
}

/* FETCH2WD: send the next two SP words of the nDNADR in progress.         */
static agc_status_t fetch2wd(void)
{
    if (!AGC_ASSERT(dn_left > 0))
    {
        return AGC_STATUS_BAD_ARG;
    }
    uint16_t a = item_word(dn_item, dn_offset);
    uint16_t l = item_word(dn_item, dn_offset + 1);
    dn_offset += 2; /* ADS DNECADR (74001)      */
    dn_left--;
    return dntmexit(a, l);
}

/* SNAPLOOP / SNAPEND: copy all but the last entry of a snapshot sublist
 * into DNTMBUFF and send the last one now.                                */
static agc_status_t snapshot(const agc_dn_list_t *sub)
{
    if (!AGC_ASSERT(sub != NULL && sub->count >= 1) ||
        !AGC_ASSERT(2 * (sub->count - 1) <= DNTMBUFF_WORDS))
    {
        return AGC_STATUS_BAD_ARG;
    }
    for (int i = 0; i + 1 < sub->count; i++)
    {
        agc_dn_entry_t e = sub->entry[i];
        if (!AGC_ASSERT(e.op == 1)) /* 1DNADRs only (rule 3)     */
        {
            return AGC_STATUS_BAD_ARG;
        }
        dntmbuff[2 * i]     = item_word((agc_dn_item_t) e.ref, e.offset);
        dntmbuff[2 * i + 1] = item_word((agc_dn_item_t) e.ref, e.offset + 1);
    }
    agc_dn_entry_t last = sub->entry[sub->count - 1];
    sublist.active      = false; /* SUBLIST, DNECADR = -1    */
    dn_left             = 0;
    return dntmexit(item_word((agc_dn_item_t) last.ref, last.offset),
                    item_word((agc_dn_item_t) last.ref, last.offset + 1));
}

/* DODNPTR: start a sublist.                                               */
static agc_status_t dodnptr(int ref)
{
    agc_dn_list_t sub;
    agc_status_t  st = list_lookup(true, ref, &sub);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    if (sub.snapshot)
    {
        return snapshot(&sub);
    }
    sublist.list     = sub; /* DOSUBLST                 */
    sublist.next     = 0;
    sublist.active   = true;
    agc_dn_entry_t e = cursor_take(&sublist);
    if (!AGC_ASSERT(e.op != AGC_DN_OP_PTR)) /* sublists do not nest     */
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (e.op == AGC_DN_OP_CHAN)
    {
        return dntmexit(agc_chan_read((int) e.ref),
                        agc_chan_read((int) e.ref + 1));
    }
    dn_item   = (agc_dn_item_t) e.ref;
    dn_offset = e.offset;
    dn_left   = e.op;
    return fetch2wd();
}

/* SETWO .. DODNCHAN: act on one list entry.  from_control: the TIME2 test
 * (word order code to zero) is made only for control-list entries.        */
static agc_status_t do_entry(agc_dn_entry_t e, bool from_control)
{
    if (!AGC_ASSERT(e.op >= 1 && e.op <= AGC_DN_OP_PTR))
    {
        return AGC_STATUS_BAD_ARG;
    }
    if (from_control && e.op == 1 && e.ref == (uint16_t) AGC_DN_TIME2)
    {
        wozero(); /* SETWO                    */
    }
    if (e.op == AGC_DN_OP_PTR)
    {
        return dodnptr((int) e.ref);
    }
    if (e.op == AGC_DN_OP_CHAN)
    {                                   /* DODNCHAN: channels n,n+1 */
        if (!AGC_ASSERT(e.ref != 0u)) /* 'DNCHAN 0' cannot be used */
        {
            return AGC_STATUS_BAD_ARG;
        }
        return dntmexit(agc_chan_read((int) e.ref),
                        agc_chan_read((int) e.ref + 1));
    }
    dn_item   = (agc_dn_item_t) e.ref;
    dn_offset = e.offset;
    dn_left   = e.op;
    return fetch2wd();
}

/* NEWLIST: start the list DNLSTCOD; the ID word is -DNLSTCOD.            */
static agc_status_t newlist(void)
{
    agc_status_t st = list_lookup(false, dnlstcod, &ctlist.list);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return st;
    }
    ctlist.next   = 0;
    ctlist.active = true;
    return sendid((uint16_t) (WORD15 - (uint16_t) dnlstcod)); /* CS DNLSTCOD */
}

/* DNPHASE2: the next word of the downlist.                                */
static agc_status_t dnphase2(void)
{
    if (dn_left > 0)
    { /* CCS DNECADR: in progress */
        return fetch2wd();
    }
    if (sublist.active)
    { /* NEXTINSL                 */
        agc_dn_entry_t e = cursor_take(&sublist);
        return do_entry(e, false);
    }
    if (!ctlist.active)
    { /* CHKLIST: end of list     */
        return newlist();
    }
    agc_dn_entry_t e = cursor_take(&ctlist); /* NEXTINCL                 */
    return do_entry(e, true);
}

/* DNPHASE1: reset the pointers and restart the current list.             */
static agc_status_t dnphase1(void)
{
    sublist.active = false;
    dn_left        = 0;
    dntmgoto       = TM_PHASE2;
    if (!AGC_ASSERT(!sublist.active))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return newlist();
}

/* ---- erasable dump (V74) ----------------------------------------------- */

/* DNDUMPI +2 (SENDID with the dump ID).                                   */
static agc_status_t dndump_id(void)
{
    dntmgoto = TM_DUMP_LOC;
    bool lm  = (agc_state_vehicle() == AGC_VEHICLE_LM);
    if (!AGC_ASSERT((dumploc & LOW8) == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    wozero();
    return dntmexit(lm ? ERASID_LM : ERASID_CSM, (uint16_t) AGC_DN_LOWIDCOD);
}

/* DNDUMP: step DUMPLOC by one DP word; at the end of an EBANK either end
 * the dump (DUMPCNT complete dumps) or send the next EBANK's ID.         */
static agc_status_t dndump(void)
{
    dumploc = (uint16_t) ((dumploc + 2u) & WORD15);
    if ((dumploc & LOW8) != 0u)
    {
        return dntmexit(0, 0); /* DNDUMP2 (DEVIATION: 0)   */
    }
    if ((dumploc & dumpcnt & PRIO34) != 0u)
    {
        return dnphase1(); /* resume the downlist      */
    }
    if (!AGC_ASSERT((dumploc & PRIO34) <= PRIO34))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return dndump_id();
}

/* ---- public ------------------------------------------------------------ */

/* DODOWNTM (DOWNRUPT, every 20 ms): word order code to 1, then the phase
 * in DNTMGOTO.                                                             */
agc_status_t agc_telem_downrupt(void)
{
    agc_chan_or((int) AGC_CHAN_CHAN13, (uint16_t) BIT7); /* WOTEST/WO1 */
    if (!AGC_ASSERT(dn_left >= 0 && dn_left <= 6))
    {
        return AGC_STATUS_BAD_ARG;
    }
    switch (dntmgoto)
    {
        case TM_PHASE1:
            return dnphase1();
        case TM_PHASE2:
            return dnphase2();
        case TM_DUMP_ID:
            return dndump_id();
        case TM_DUMP_LOC: /* DUMPLOC and TIME1        */
            dntmgoto = TM_DUMP_FIRST;
            return dntmexit(dumploc, sp_word(agc_erase_time1()));
        case TM_DUMP_FIRST: /* DNDUMP1 -> DNDUMP2       */
            dntmgoto = TM_DUMP_NEXT;
            return dntmexit(0, 0);
        case TM_DUMP_NEXT:
            return dndump();
        default:
            break;
    }
    return AGC_STATUS_BAD_ARG;
}

/* Fresh start (DOFSTART): DNLSTCOD (and AGSWORD on the LM) = the P00
 * downlist; LM DUMPCNT = BIT12 (one pass); DNTMGOTO = DNPHASE1.          */
agc_status_t agc_telem_fresh_start(int code)
{
    agc_telem_select_list(code);
    if (agc_state_vehicle() == AGC_VEHICLE_LM)
    {
        agc_erase_set_agsword((uint16_t) code);
        dumpcnt = 004000u; /* BIT12: dump for one pass */
    }
    dumploc = 0;
    return agc_telem_restart();
}

/* STARTSUB: the next DOWNRUPT restarts the current downlist.              */
agc_status_t agc_telem_restart(void)
{
    dntmgoto = TM_PHASE1;
    dn_left  = 0;
    if (!AGC_ASSERT(dntmgoto == TM_PHASE1))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

/* TS DNLSTCOD: select the list sent from the next NEWLIST on.            */
void agc_telem_select_list(int code)
{
    int n = (agc_state_vehicle() == AGC_VEHICLE_LM) ? AGC_DNLIST_LM_COUNT :
                                                      AGC_DNLIST_CSM_COUNT;
    if (!AGC_ASSERT(code >= 0 && code < n))
    {
        return;
    }
    dnlstcod = code;
}

int agc_telem_list_code(void)
{
    if (!AGC_ASSERT(dnlstcod >= 0 && dnlstcod < AGC_DNLIST_LM_COUNT))
    {
        return 0; /* no valid result */
    }
    return dnlstcod;
}

/* V74 DNEDUMP: DNTMGOTO = DNDUMPI (DUMPLOC = 0).                          */
agc_status_t agc_telem_start_dump(void)
{
    dumploc  = 0;
    dntmgoto = TM_DUMP_ID;
    if (!AGC_ASSERT(dumploc == 0u))
    {
        return AGC_STATUS_BAD_ARG;
    }
    return AGC_STATUS_OK;
}

agc_status_t agc_telem_set_dumpcnt(uint16_t v)
{
    if (!AGC_ASSERT(v <= WORD15))
    {
        return AGC_STATUS_BAD_ARG;
    }
    dumpcnt = v;
    return AGC_STATUS_OK;
}

uint16_t agc_telem_dumpcnt(void)
{
    if (!AGC_ASSERT(dumpcnt <= WORD15))
    {
        return 0; /* no valid result */
    }
    return dumpcnt;
}

/* AGSWORD (LM): the downlist to return to after AGS initialization.      */
uint16_t agc_erase_agsword(void)
{
    if (!AGC_ASSERT(agsword < (uint16_t) AGC_DNLIST_LM_COUNT))
    {
        return 0; /* no valid result */
    }
    return agsword;
}

void agc_erase_set_agsword(uint16_t v)
{
    if (AGC_ASSERT(v < (uint16_t) AGC_DNLIST_LM_COUNT))
    {
        agsword = v;
    }
}

/* DP words one DNPTR sends at its place in the list: a snapshot sublist
 * sends only its last entry now (the rest go out via DNTMBUFF entries).   */
static int sublist_words(int ref)
{
    agc_dn_list_t sub;
    agc_status_t  st = list_lookup(true, ref, &sub);
    if (!AGC_ASSERT(st == AGC_STATUS_OK))
    {
        return 0;
    }
    if (sub.snapshot)
    {
        return 1;
    }
    int n = 0;
    for (int i = 0; i < sub.count; i++)
    {
        uint8_t op = sub.entry[i].op;
        if (!AGC_ASSERT(op != AGC_DN_OP_PTR))
        {
            return 0;
        }
        n += (op == AGC_DN_OP_CHAN) ? 1 : (int) op;
    }
    return n;
}

/* Number of DP words a downlist sends, the ID word included.             */
agc_status_t agc_dnlist_word_count(agc_vehicle_t v, int code, int *words)
{
    if (!AGC_ASSERT(words != NULL) || !AGC_ASSERT(v == agc_state_vehicle()))
    {
        return AGC_STATUS_BAD_ARG;
    }
    agc_dn_list_t list;
    agc_status_t  st = list_lookup(false, code, &list);
    if (st != AGC_STATUS_OK)
    {
        return st;
    }
    int n = 1; /* ID / sync word           */
    for (int i = 0; i < list.count; i++)
    {
        agc_dn_entry_t e = list.entry[i];
        n += (e.op == AGC_DN_OP_PTR)  ? sublist_words((int) e.ref) :
             (e.op == AGC_DN_OP_CHAN) ? 1 :
                                        (int) e.op;
    }
    *words = n;
    return AGC_STATUS_OK;
}
