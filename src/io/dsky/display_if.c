/*
 * display_if.c -- DISPLAY_INTERFACE_ROUTINES.agc (CM pp. 1452-1484, LM pp.
 * 1341-1373): the interface between mission programs and PINBALL.
 *
 * "DISPLAYS CAN BE CLASSIFIED INTO THE FOLLOWING CATEGORIES-
 *  1. PRIORITY DISPLAYS ... 2. EXTENDED VERB DISPLAYS ... (MARK) ...
 *  3. NORMAL DISPLAYS ..."  Priority displays interrupt all other internal
 * displays; a mark display interrupts any normal display; the astronaut
 * can interrupt any display with an external display request. Order of
 * waiting displays: astronaut, priority, interrupted mark, interrupted
 * normal, mark to be requested, mark waiting, normal waiting.
 *
 * The three display classes are indexed by COPINDEX: 0 priority, 1 mark,
 * 2 normal. Their state is in FLAGWRD4 (idle, waiting, NV-busy and
 * interrupted bits) and in the per-class copies NVWORD/MARKNV/NVSAVE,
 * CADRFLSH/CADRMARK/TEMPFLSH, EBANKSAV/MARKEBAN/EBANKTEM (the display
 * flags, DSPFLG). Routines not ending in R run in the user's job; R
 * routines start job MAKEPLAY at the user's priority and return at once.
 *
 * C model: a job that the AGC JOBSLEEPs or JOBWAKEs at another CADR is put
 * to sleep / woken with the continuation job ids of internal.h;
 * the user's returns (call CADR +1/+2/+3) are the agc_dsky_cont_t ids,
 * reached by agc_exec_job_yield() after PRIOCHNG to the user's priority
 * (ENDIT). The job temporaries the AGC keeps in the sleeping job's MPAC
 * and MODE across ENDIDLE (COPMPAC, GENMASK, USERPRIO) are kept here in
 * g_idle, since only one job can be in ENDIDLE.
 */
#include <stddef.h>

#include "pinball/internal.h"

void agc_erase_set_extvbact(uint16_t v); /* EXTVBACT (sys-misc)  */

/* ---- FLAGWRD4 bits (display flags) ------------------------------------ */
#define DIF_F4       4
#define DIF_MRKIDFLG 040000u
#define DIF_PRIODFLG 020000u
#define DIF_NRMIDFLG 010000u
#define DIF_MWAITFLG 002000u
#define DIF_NWAITFLG 001000u
#define DIF_MRKNVFLG 000400u
#define DIF_NRMNVFLG 000200u
#define DIF_PRONVFLG 000100u
#define DIF_PINBRFLG 000040u
#define DIF_MRUPTFLG 000020u
#define DIF_NRUPTFLG 000010u
#define DIF_MKOVFLAG 000004u
#define DIF_XDSPFLAG 000001u
/* masks (the AGC's UPENT2/DOWNENT2 words without the flagword number 4)  */
#define DIF_WAITMASK 003000u
#define DIF_IDLEMASK 074000u
#define DIF_ASTROMSK 000700u
#define DIF_IDLESLEP 074700u
#define DIF_PINIDMSK 074040u
#define DIF_NVBUSMSK 000700u
#define DIF_NBUSMASK 011210u
#define DIF_PMMASK   066521u
#define DIF_OCT34300 034300u
#define DIF_OCT24100 024100u
#define DIF_OCT40400 040400u
#define DIF_OCT10200 010200u
#define DIF_OCT20100 020100u
#define DIF_ITISMASK 040040u
#define DIF_PINMASK  070000u
#define DIF_OCT60000 060000u
#define DIF_PRIO30   030000u
#define DIF_BITS5_11 002020u
#define DIF_BITS4_10 001010u
/* PLAYTEM4 / DSPFLG display flag bits                                     */
#define DIF_MARK      040000u /* bit 15 mark request                    */
#define DIF_LINUS     020000u /* bit 14 replace an active display       */
#define DIF_SECOND    010000u /* bit 13 first part of a perform shown   */
#define DIF_CLOCK     004000u /* bit 12 V97/V99 paste                   */
#define DIF_DECMARK   000200u /* bit 8 decimal mark perform             */
#define DIF_PRIO      000100u /* bit 7 priority                         */
#define DIF_GODSPRET  000040u /* bit 6 return to user after display     */
#define DIF_PERFORM   000020u /* bit 5 please perform                   */
#define DIF_FLASH     000010u /* bit 4 flash                            */
#define DIF_BLANKS    000007u /* bits 1-3 blank R1, R2, R3              */
#define DIF_CADRMASK  000050u /* flash or GODSPRET                      */
#define DIF_EBANKBITS 003400u /* OCT3400 (no meaning in C)              */
#define DIF_2SEC      200     /* -2SEC DEC -200 (centiseconds)          */
#define DIF_RQ_LEN    4       /* R requests waiting for their MAKEPLAY  */

enum
{
    DIF_PRIOC = 0,
    DIF_MARKC = 1,
    DIF_NORMC = 2
}; /* COPINDEX       */

/* PRIOOCT, MARKOCT, NORM: the FLAGWRD4 bits that belong to each class. */
static const uint16_t k_genmask[3] = {020140u, 042420u, 011250u};

typedef struct dif_class
{
    uint16_t        flags;    /* EBANKSAV / MARKEBAN / EBANKTEM (DSPFLG)  */
    agc_dsky_cont_t cadr;     /* CADRFLSH / CADRMARK / TEMPFLSH           */
    uint16_t        nvword;   /* NVWORD / MARKNV / NVSAVE (ones' compl.)  */
    agc_prio_t      userprio; /* USERPRIO of the requesting job (MODE)    */
} dif_class_t;

typedef struct dif_req
{ /* PLAYTEM1, PLAYTEM3, PLAYTEM4             */
    bool            used;
    uint16_t        vn;
    uint16_t        flags;
    agc_dsky_cont_t cadr;
} dif_req_t;

typedef struct dif_job
{                        /* job temporaries (LOC, MPAC +3..+6)       */
    int        c;        /* COPINDEX                                 */
    uint16_t   tempor2;  /* TEMPOR2: flags copy                      */
    agc_prio_t userprio; /* USERPRIO                                 */
    int        outhere;  /* OUTHERE: 0 term, 1 proceed, 2 enter, -1 end */
} dif_job_t;

static dif_class_t g_cls[3];
static dif_req_t   g_rq[DIF_RQ_LEN];
static int         g_rq_last = -1; /* MPAC+5 of the last R request      */
static uint16_t    g_priotime;     /* PRIOTIME (TIME1)                  */
static uint16_t    g_nvword1 = PIN_NEGZERO; /* NVWORD1                     */
static agc_prio_t  g_restreg; /* RESTREG: normal user's priority   */

static struct
{
    int        copmpac;
    agc_prio_t userprio;
} g_idle;

/* ---- helpers ---------------------------------------------------------- */

static uint16_t f4(void)
{
    return (agc_flagword_get(DIF_F4));
}

/* UPENT2 / DOWNENT2 on FLAGWRD4 (UPFLAG / DOWNFLAG for one bit). */
static void f4_up(uint16_t bits)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return;
    }

    agc_inhint();
    agc_flagword_set(DIF_F4, (uint16_t) (f4() | bits)); /* cannot fail for word 4 */
    agc_relint();
}

static void f4_down(uint16_t bits)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return;
    }

    agc_inhint();
    agc_flagword_set(DIF_F4, (uint16_t) (f4() & ~bits));
    agc_relint();
}

static bool dif_cont_valid(const agc_dsky_cont_t *c)
{
    return (c->terminate != AGC_JOB_NONE && c->proceed != AGC_JOB_NONE &&
            c->enter != AGC_JOB_NONE);
}

/* PRIOBORT: abort 01502, display conflict. */
static agc_status_t dif_priobort(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_poodoo(PIN_ALARM_PRIOBORT));
}

/* WITCHONE: turn off key release light and empty CADRSTOR (or DSPLIST if
 * an NVSUB is asleep); the job there is woken at new_loc (XCHTOEND:
 * ENDINST, i.e. it ends; JOBXCHS: XCHSLEEP).                            */
static agc_status_t dif_witchone(agc_job_id_t new_loc)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_chan_and((int) AGC_CHAN_DSALMOUT,
                 (uint16_t) (PIN_WORD ^ PIN_CH11_KEYREL));
    pin_sleeper_t sl = pin_take_sleeper((f4() & DIF_NVBUSMSK) != 0u);
    agc_status_t  st = AGC_STATUS_OK;

    agc_inhint();

    if (sl.used)
    {
        st = agc_exec_job_wake_as(sl.cont[0], new_loc); /* XCHNYLOC  */
        st = (st == AGC_STATUS_NOT_FOUND) ? AGC_STATUS_OK : st;
    }

    agc_relint(); /* RELINTQ   */

    if (!AGC_ASSERT(new_loc != AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* COPIES / COPIES2: copy the request into class c. */
static void dif_copies(int c, const dif_req_t *rq, agc_prio_t userprio)
{
    if (!AGC_ASSERT(c >= DIF_PRIOC && c <= DIF_NORMC) ||
        !AGC_ASSERT(rq != NULL))
    {
        return;
    }

    agc_inhint();
    g_cls[c].flags = (uint16_t) (rq->flags & ~DIF_EBANKBITS); /* DSPFLG   */

    if ((rq->flags & DIF_CADRMASK) != 0u)
    {
        g_cls[c].cadr = rq->cadr; /* CADRFLSH */
    }

    g_cls[c].nvword   = rq->vn; /* NVWORD   */
    g_cls[c].userprio = userprio;
    agc_relint();
}

/* ENDIT: return to the user's priority and to his CADR. */
static agc_status_t dif_endit(agc_prio_t userprio, agc_job_id_t cont)
{
    if (!AGC_ASSERT(cont != AGC_JOB_NONE) ||
        !AGC_ASSERT(userprio >= 1u && userprio <= 037u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_exec_prio_change(userprio); /* BUSY only means: yield, done next */

    return (agc_exec_job_yield(cont));
}

static agc_status_t dif_nvdsp(int c, agc_prio_t userprio);

/* ---- sleeping displays (GOSLEEPS, XCHSLEEP) --------------------------- */

/* XCHSLEEP: a newer display of the class replaces any older one asleep at
 * its WAKECADR (that job is woken to ENDOFJOB); then this job sleeps. */
static agc_status_t dif_xchsleep(int c)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    /* WAKECADR: where a sleeping mark or normal display job wakes. */
    if (!AGC_ASSERT(c == DIF_MARKC || c == DIF_NORMC))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_job_id_t wake = (c == DIF_MARKC) ? AGC_JOB_MARKPLAY : AGC_JOB_PLAYJUM1;

    agc_inhint();
    agc_status_t st = agc_exec_job_wake_as(wake, AGC_JOB_ENDOFJOB);

    agc_relint();

    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return (st);
    }

    agc_exec_job_sleep(wake);

    return (AGC_STATUS_OK); /* JOBSLEEP   */
}

/* GOSLEEPS: set the class's waiting bit, then XCHSLEEP. */
static agc_status_t dif_gosleeps(int c)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(c == DIF_MARKC || c == DIF_NORMC))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    f4_up((uint16_t) (k_genmask[c] & DIF_WAITMASK));

    return (dif_xchsleep(c));
}

agc_status_t agc_job_xchsleepm(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_xchsleep(DIF_MARKC));
}

agc_status_t agc_job_xchsleepn(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_xchsleep(DIF_NORMC));
}

/* ---- the three display classes (MAKEPRIO, MAKEMARK, IFLEGAL) ---------- */

/* REDOPRIO / KEEPPRIO: start up the priority display. */
static agc_status_t dif_redoprio(agc_prio_t userprio)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    g_priotime = (uint16_t) agc_erase_time1(); /* save time PRIODSP sent out */

    return (dif_nvdsp(DIF_PRIOC, userprio));
}

/* OKTOCOPY: copy into the priority class, end whatever display job sits
 * in CADRSTOR/DSPLIST, and go to REDOPRIO.                              */
static agc_status_t dif_oktocopy(const dif_req_t *rq, agc_prio_t userprio)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (rq != NULL)
    {
        dif_copies(DIF_PRIOC, rq, userprio); /* COPYNORM               */
    }

    agc_status_t st = dif_witchone(AGC_JOB_ENDOFJOB);

    return ((st ==
             AGC_STATUS_OK) ? dif_redoprio(g_cls[DIF_PRIOC].userprio) : st);
}

/* MARKPLAY: reset mark over norm, set mark (XDSPFLAG); for a mark perform
 * the first display is V05/V06 (MARKNV complemented as the marker).     */
static agc_status_t dif_markplay(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_flagword_set(DIF_F4, (uint16_t) ((f4() & ~05u) | DIF_XDSPFLAG));
    agc_relint();

    if ((g_cls[DIF_MARKC].flags & DIF_PERFORM) != 0u)
    { /* GOGOMARK */
        g_cls[DIF_MARKC].nvword = pin_ones_neg(g_cls[DIF_MARKC].nvword);
    }

    if (!AGC_ASSERT((f4() & DIF_XDSPFLAG) != 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_nvdsp(DIF_MARKC, g_cls[DIF_MARKC].userprio)); /* MARKCOP */
}

/* JOBXCHS: a priority (or mark) display interrupts the active mark (face
 * 0) or normal (face 1) display: its job is moved from CADRSTOR/DSPLIST
 * to sleep at its WAKECADR, its idle/NV-busy bits become interrupted.  */
static agc_status_t dif_jobxchs(int face, const dif_req_t *rq,
                                agc_prio_t userprio)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(face == 0 || face == 1))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st =
        dif_witchone((face == 0) ? AGC_JOB_XCHSLEEPM : AGC_JOB_XCHSLEEPN);

    f4_down((uint16_t) (k_genmask[face + 1] & DIF_IDLESLEP));
    f4_up((face == 0) ? DIF_MRUPTFLG : DIF_NRUPTFLG);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if ((f4() & DIF_MKOVFLAG) != 0u)
    {                          /* mark over norm         */
        return (dif_markplay()); /* GENMARK                */
    }

    return (dif_oktocopy(rq, userprio));
}

/* LINUSCHR: a request with bit 14 replaces the active display directly
 * (LINUS return, *linus). If it is the display already in ENDIDLE and the
 * astronaut is busy, the new display just ends (*end).                  */
static void dif_linuschr(int c, const dif_req_t *rq, bool *linus, bool *end)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return;
    }

    *linus = false;
    *end   = false;

    if ((rq->flags & DIF_LINUS) == 0u)
    {
        return; /* Q+1: no                */
    }

    *linus = true;

    if (rq->cadr.terminate == g_cls[c].cadr.terminate && pin_st()->dsplock)
    {
        *end = true; /* already active         */
    }

    if (!AGC_ASSERT(c >= DIF_PRIOC && c <= DIF_NORMC))
    {
        return;
    }
}

/* MAKEPRIO: a priority display interrupting another priority display
 * aborts (01502) unless bit 14 is set for the LINUS routine.            */
static agc_status_t dif_makeprio(const dif_req_t *rq, agc_prio_t userprio)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool linus = false;
    bool end   = false;

    dif_linuschr(DIF_PRIOC, rq, &linus, &end);

    if (end)
    {
        return (AGC_STATUS_OK);
    }

    if (!linus && (f4() & DIF_OCT20100) != 0u)
    { /* prio in ENDIDLE or busy */
        return (dif_priobort());
    }

    if ((f4() & DIF_OCT40400) != 0u)
    {                                        /* HIPRIO: mark active    */
        return (dif_jobxchs(0, rq, userprio)); /* SETMARK                */
    }

    if ((f4() & DIF_OCT10200) != 0u)
    {                                        /* normal active          */
        return (dif_jobxchs(1, rq, userprio)); /* SETNORM                */
    }

    return (dif_oktocopy(rq, userprio));
}

/* MAKEMARK / CHKPRIO / MARSLEEP. */
static agc_status_t dif_makemark(const dif_req_t *rq, agc_prio_t userprio)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    dif_copies(DIF_MARKC, rq, userprio);
    uint16_t f = f4();

    if ((f & DIF_OCT34300) == 0u)
    { /* norm or prio busy/waiting? */
        if ((f & DIF_MRKNVFLG) != 0u)
        {
            return (AGC_STATUS_OK); /* mark asleep, astro busy */
        }

        return (dif_markplay());
    }

    if ((f & DIF_OCT24100) != 0u)
    { /* mark attempt during prio */
        if ((f & DIF_BITS5_11) != 0u)
        {
            return (AGC_STATUS_OK); /* mark already on        */
        }

        return (dif_gosleeps(DIF_MARKC));
    }

    f4_up(DIF_MKOVFLAG);                   /* set mark over norm     */

    return (dif_jobxchs(1, NULL, userprio)); /* SETNORM                */
}

/* IFLEGAL / OKTOPLAY: a normal display. A new one while the previous
 * flashing normal display is still asleep aborts (01502).              */
static agc_status_t dif_makenorm(const dif_req_t *rq, agc_prio_t userprio)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool linus = false;
    bool end   = false;

    dif_linuschr(DIF_NORMC, rq, &linus, &end);

    if (end)
    {
        return (AGC_STATUS_OK);
    }

    if (!linus && (g_cls[DIF_NORMC].flags & DIF_FLASH) != 0u &&
        (f4() & DIF_NBUSMASK) != 0u)
    {
        return (dif_priobort()); /* were norms asleep      */
    }

    dif_copies(DIF_NORMC, rq, userprio); /* OKTOPLAY: COPIES2      */
    g_restreg = userprio;

    if ((f4() & DIF_PMMASK) != 0u)
    { /* prio or mark going     */
        return (dif_gosleeps(DIF_NORMC));
    }

    agc_status_t st = dif_witchone(AGC_JOB_ENDOFJOB);

    return ((st == AGC_STATUS_OK) ? dif_nvdsp(DIF_NORMC, userprio) :
            st);                       /* PLAYJUM1 */
}

/* MAKEPLAY: is it mark or prio or norm. The priority is raised to 33 for
 * fast jobs after wake.                                                  */
static agc_status_t dif_makeplay(const dif_req_t *rq, agc_prio_t userprio)
{
    if (!AGC_ASSERT(rq != NULL) ||
        !AGC_ASSERT(userprio >= 1u && userprio <= 037u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_exec_prio_change(PIN_PRIO33); /* raising: never pre-empted */

    if ((rq->flags & DIF_MARK) != 0u)
    {
        return (dif_makemark(rq, userprio));
    }

    if ((rq->flags & DIF_PRIO) != 0u)
    {
        return (dif_makeprio(rq, userprio));
    }

    return (dif_makenorm(rq, userprio));
}

/* ---- NVDSP / PERFCHEK / FLASHSUB (pp. 1472-1477) ---------------------- */

/* CLEANEND: a display with VN +0 (CLEANDSP, KLEENEX) ends any ENDIDLE
 * through JAMTERM, one priority lower than the displays sleeping. */
static agc_status_t dif_cleanend(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_exec_findvac(PIN_PRIO32, AGC_JOB_JAMTERM);

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_FULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (st);
}

/* FLASHSUB: flash on (unless +1), set the class's ENDIDLE bit and wait in
 * ENDIDLE for the astronaut. If someone already is in ENDIDLE, only a
 * pinbranch or mark condition may get here; anything else aborts 01502.*/
static agc_status_t dif_flashsub(int c, agc_prio_t userprio, bool flash)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(c >= DIF_PRIOC && c <= DIF_NORMC))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (flash)
    {
        pin_flashon();
    }

    f4_up((uint16_t) (k_genmask[c] & DIF_IDLEMASK));

    /* DEVIATION: the R1SAVE repeat-and-return branch (UNSETR1) is not
     * modelled: R1SAVE is only ever zeroed (fresh start) in both programs. */
    if (pin_cadrstor_full())
    {
        return (((f4() & DIF_ITISMASK) == 0u) ? dif_priobort() :
                AGC_STATUS_OK);                               /* ISITPRIO */
    }

    g_idle.copmpac  = c; /* COPMPAC                */
    g_idle.userprio = userprio;

    return (pin_endidle(AGC_JOB_TERMATE, AGC_JOB_PROCEED, AGC_JOB_IDLERET3));
}

/* REST: the display system is busy. If someone is in ENDIDLE the display
 * is dropped (it is saved for PINBRNCH); else RESTSLEP: NV-busy bit and
 * sleep in DSPLIST, to wake at REDOPRIO / MARKPLAY / PLAYJUM1 (NVCADR). */
static agc_status_t dif_rest(int c)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    static const agc_job_id_t nvcadr[3] = {AGC_JOB_REDOPRIO, AGC_JOB_MARKPLAY,
                                           AGC_JOB_PLAYJUM1};

    if (!AGC_ASSERT(c >= DIF_PRIOC && c <= DIF_NORMC))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (pin_cadrstor_full())
    {
        return (AGC_STATUS_OK); /* ENDOFJOB               */
    }

    f4_up((uint16_t) (k_genmask[c] & DIF_ASTROMSK));
    agc_status_t st = pin_nvsubusy(nvcadr[c]); /* busy or abort if illegal */

    return ((st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st);
}

/* GOANIDLE: flash -> FLASHSUB; GODSPRET -> back to the user's CADR; a
 * paste (noun 00) is assumed always on flash; else ENDOFJOB.             */
static agc_status_t dif_goanidle(const dif_job_t *j)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if ((j->tempor2 & DIF_FLASH) != 0u)
    {
        return (dif_flashsub(j->c, j->userprio, true));
    }

    if ((j->tempor2 & DIF_GODSPRET) != 0u)
    {
        return (dif_endit(j->userprio, g_cls[j->c].cadr.terminate)); /* ENDIT */
    }

    if ((g_cls[j->c].nvword & PIN_LOW7) == 0u)
    { /* ISITN00: is this a paste */
        return (dif_flashsub(j->c, j->userprio, true));
    }

    if (!AGC_ASSERT(j->c >= DIF_PRIOC))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* not flash, not GOPERF: exit      */
}

/* 1STOR2ND: the VN of the second part of a please perform: the mark's
 * please verb, V97/V99 (NVWORD1 -0 / -400) for CLOCPLAY, else V50 N00. */
static uint16_t dif_second_vn(const dif_job_t *j)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (0); /* no valid result */
    }

    if ((j->tempor2 & DIF_MARK) != 0u)
    {
        return ((uint16_t) (g_cls[DIF_MARKC].nvword & PIN_MID7)); /* MARKPERF  */
    }

    if ((j->tempor2 & DIF_CLOCK) != 0u)
    {
        int vn = (int) agc_dsky_vn(97, 0) - (int) pin_sp_from_ones(g_nvword1);

        if (!AGC_ASSERT(vn > 0 && vn <= (int) PIN_POSMAX))
        {
            return (0); /* no valid result */
        }

        return ((uint16_t) vn);
    }

    return (agc_dsky_vn(50, 0)); /* V50PASTE  */
}

/* First VN of NVDSP for class c; 0 for CLEANEND. A negative MARKNV is
 * the first part of a mark perform: V05 (V06 if decimal) and its noun. */
static uint16_t dif_first_vn(int c)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (0); /* no valid result */
    }

    uint16_t nv = g_cls[c].nvword;

    if (!pin_ones_negative(nv))
    {
        return (nv);
    }

    g_cls[c].nvword =
        pin_ones_neg(nv); /* in case MARKPLAY awakened after sleeping */
    uint16_t dec = ((g_cls[c].flags & DIF_DECMARK) != 0u) ? 1u : 0u;

    return ((uint16_t) ((g_cls[c].nvword & PIN_LOW7) +
                        agc_dsky_vn((uint8_t) (5u + dec), 0)));
}

/* NVDSP (GOPLAY): send the display of class c through NVMONOPT, then the
 * second part of a perform, then GOANIDLE.                              */
static agc_status_t dif_nvdsp(int c, agc_prio_t userprio)
{
    if (!AGC_ASSERT(c >= DIF_PRIOC && c <= DIF_NORMC))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    dif_job_t j    = {c, g_cls[c].flags, userprio, 0}; /* COPYPACS */
    uint16_t  opts =
        (uint16_t) (j.tempor2 & DIF_BLANKS); /* blank bits for NVMONOPT */

    g_cls[c].flags = (uint16_t) (g_cls[c].flags & ~DIF_SECOND);
    uint16_t vn = dif_first_vn(c);

    if (vn == 0u)
    {
        agc_status_t st = dif_cleanend();

        return ((st == AGC_STATUS_OK) ? dif_flashsub(c, userprio, false) :
                st);                       /* FLASHSUB +1 */
    }

    for (int part = 0; part < 2; part++)
    { /* NV50DSP  */
        agc_status_t st = pin_nvmonopt(vn, opts);

        if (st == AGC_STATUS_BUSY)
        {
            return (dif_rest(c));
        }

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }

        pin_flashoff();             /* in case of extended verb non flash */
        j.tempor2 = g_cls[c].flags; /* COPYTOGO: MPACS destroyed by NVSUB */
        f4_down(DIF_MRKNVFLG | DIF_NRMNVFLG |
                DIF_PRONVFLG);        /* unset sleeping bits */
        st = pin_blanksub(j.tempor2); /* BLANKCHK                      */

        if (!AGC_ASSERT(st != AGC_STATUS_BUSY))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if ((j.tempor2 & DIF_PERFORM) == 0u || (j.tempor2 & DIF_SECOND) != 0u)
        {
            break; /* PERFCHEK: not GOPERF, or second */
        }

        g_cls[c].flags = (uint16_t) (g_cls[c].flags | DIF_SECOND);
        vn             = dif_second_vn(&j);
        opts           = 0; /* ZL                            */
    }

    return (dif_goanidle(&j));
}

/* ---- ENDIDLE returns (pp. 1477-1481) ---------------------------------- */

/* ENDRET: return to the user's CADR +OUTHERE after removing the ENDIDLE
 * and pinbranch bits and blanking everything except MM (NVSUB -3).      */
static agc_status_t dif_endret(int outhere)
{
    int c = g_idle.copmpac;

    if (outhere < 0)
    {
        return (AGC_STATUS_OK); /* ENDOFJOB (MARKOVER)    */
    }

    if (!AGC_ASSERT(c >= DIF_PRIOC && c <= DIF_NORMC) ||
        !AGC_ASSERT(outhere <= 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const agc_dsky_cont_t *k    = &g_cls[c].cadr;
    agc_job_id_t           cont = (outhere == 0) ?
                                  k->terminate :
                                  ((outhere == 1) ? k->proceed : k->enter);

    f4_down((uint16_t) (k_genmask[c] & DIF_PINIDMSK));
    agc_status_t st = agc_dsky_nvsub_blank(-3);

    if (st != AGC_STATUS_OK && st != AGC_STATUS_BUSY)
    {
        return (st);
    }

    return (dif_endit(g_idle.userprio, cont));
}

/* NORMRET: wake a mark or normal display that is asleep (interrupted or
 * waiting), or redisplay the last non-flashing normal display. */
static agc_status_t dif_normret(int outhere)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t     f  = f4();
    agc_status_t st = AGC_STATUS_OK;

    if ((f & DIF_BITS5_11) != 0u)
    { /* MARKWAKE       */
        f4_down(DIF_BITS5_11);
        st = agc_exec_job_wake(AGC_JOB_MARKPLAY);
    }
    else if ((f & DIF_BITS4_10) != 0u)
    { /* NORMWAKE       */
        f4_down(DIF_BITS4_10);
        st = agc_exec_job_wake(AGC_JOB_PLAYJUM1);
    }
    else if ((g_cls[DIF_NORMC].flags & DIF_CADRMASK) == 0u &&
             g_cls[DIF_NORMC].nvword != 0u)
    {
        agc_inhint();
        st = agc_exec_novac(PIN_PRIO15, AGC_JOB_PLAYJUM1);
        agc_relint();
    }

    if (!AGC_ASSERT(outhere >= -1 && outhere <= 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (st != AGC_STATUS_OK && st != AGC_STATUS_NOT_FOUND)
    {
        return (st);
    }

    return (dif_endret(outhere));
}

/* ENDOUT: check nature of ENDIDLE return. A priority display answered
 * within 2 s of being sent out is redisplayed (TIMECHEK / KEEPPRIO).    */
static agc_status_t dif_endout(int outhere)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t m = (uint16_t) (f4() & DIF_OCT60000);

    if ((m & DIF_MRKIDFLG) != 0u)
    { /* MARKRET        */
        f4_down(06u);

        return (dif_endret(outhere));
    }

    if (m != 0u)
    { /* TIMECHEK       */
        /* CS TIME1, AD PRIOTIME: equal times give -0, i.e. no time.     */
        int d       = (int) g_priotime - (int) agc_erase_time1();
        int elapsed = (d > 0) ? 040000 - d : -d; /* TIME1 wraps at 2^14 */

        if (elapsed <= DIF_2SEC)
        {
            return (dif_redoprio(g_cls[DIF_PRIOC].userprio)); /* KEEPPRIO */
        }
    }

    if (!AGC_ASSERT(outhere >= 0 && outhere <= 2))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_normret(outhere));
}

/* TERMATE: astronaut terminate (V34) returns to call CADR +1. */
agc_status_t agc_job_termate(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_endout(0));
}

/* PROCEED: astronaut proceed (V33) returns to call CADR +2. */
agc_status_t agc_job_proceed(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_endout(1));
}

/* IDLERET1 +2: enter or recycle. A V21-V23 load of a noun other than the
 * one displayed is accepted but the display is asked for again
 * (LOADITIS -> PINBRNCH).                                                */
agc_status_t agc_job_idleret3(agc_job_ctx_t *ctx)
{
    uint8_t verb = 0;
    uint8_t noun = 0;

    if (!AGC_ASSERT(ctx != NULL) ||
        agc_dsky_response(&verb, &noun) != AGC_STATUS_OK)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    int d = (int) verb - PIN_LOWLOAD;

    if (d >= -1 && d <= 1)
    { /* V21, V22, V23   */
        int c = g_idle.copmpac;

        if (!AGC_ASSERT(c >= DIF_PRIOC && c <= DIF_NORMC))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if ((g_cls[c].nvword & PIN_LOW7) != noun)
        {
            return (pin_pinbrnch());
        }
    }

    return (dif_endout(2)); /* OKTOENT         */
}

/* ---- PINBRNCH, MARKOVER (pp. 1468-1470, 1480) ------------------------- */

/* NORMBNCH: set the pinbranch bit and redisplay the priority display (if
 * one was interrupted) or the normal one.                               */
static agc_status_t dif_normbnch(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    f4_up(DIF_PINBRFLG);

    if ((f4() & DIF_PRIODFLG) != 0u)
    {
        return (dif_redoprio(g_cls[DIF_PRIOC].userprio)); /* KEEPPRIO   */
    }

    return (dif_nvdsp(DIF_NORMC, g_cls[DIF_NORMC].userprio)); /* PLAYJUM1   */
}

/* PINBRNCH (GOAGIN): re-establish the display in ENDIDLE after the
 * astronaut used the DSKY; nothing in ENDIDLE blanks all but MM.       */
agc_status_t pin_pinbrnch(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint16_t m = (uint16_t) (f4() & DIF_PINMASK);

    if ((m & DIF_MRKIDFLG) != 0u)
    {
        return (dif_markplay());
    }

    if (m != 0u)
    {
        return (dif_normbnch());
    }

    agc_status_t st = agc_dsky_nvsub_blank(-3); /* ERASER         */

    if (!AGC_ASSERT(st == AGC_STATUS_OK || st == AGC_STATUS_BUSY ||
                    st == AGC_STATUS_POODOO))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return ((st == AGC_STATUS_BUSY) ? AGC_STATUS_OK : st);
}

agc_status_t agc_dsky_pinbrnch(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (agc_interrupts_inhibited())
    {
        agc_relint(); /* RELINT (for GOPIN users; a no-op when enabled) */
    }

    return (pin_pinbrnch());
}

/* CLEARMRK: release the mark / extended verb display system. */
void agc_dsky_clearmrk(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return;
    }

    agc_erase_set_extvbact(0);
    f4_down(DIF_XDSPFLAG);

    if (!AGC_ASSERT((f4() & DIF_XDSPFLAG) == 0u))
    {
        return;
    }
}

/* ENDMARK / ENDEXT -> MARKEND: CLEARMRK, then MARKOVER: wake the next
 * display; the extended verb's job ends (OUTHERE -1).                  */
agc_status_t agc_dsky_endmark(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_dsky_clearmrk();

    if ((f4() & DIF_PRIO30) != 0u)
    { /* normal or prio in ENDIDLE */
        return (dif_normbnch());
    }

    return (dif_normret(-1));
}

/* ENDEXT EQUALS ENDMARK (end of an extended verb). */
agc_status_t agc_dsky_endext(void)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (agc_dsky_endmark());
}

/* ---- jobs ------------------------------------------------------------- */

/* MAKEPLAY as the job an R routine started; its request is in slot arg. */
agc_status_t agc_job_makeplay(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(ctx->arg >= 1 && ctx->arg <= DIF_RQ_LEN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    dif_req_t rq = g_rq[ctx->arg - 1];

    g_rq[ctx->arg - 1].used = false;

    if (!AGC_ASSERT(rq.used))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_makeplay(&rq, ctx->prio));
}

/* PLAYJUM1: (re)display the normal class (wake point of a normal display). */
agc_status_t agc_job_playjum1(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_nvdsp(DIF_NORMC, g_cls[DIF_NORMC].userprio));
}

agc_status_t agc_job_markplay(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_markplay());
}

agc_status_t agc_job_redoprio(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_redoprio(g_cls[DIF_PRIOC].userprio));
}

/* ---- entries (pp. 1465-1467) ------------------------------------------ */

typedef struct dif_kind
{
    uint16_t flags; /* PLAYTEM4                                  */
    bool     r;     /* immediate return: separate MAKEPLAY job   */
    uint8_t  src;   /* 0 VN in A, 1 VN 0, 2 GOPERF1, 3 GOPERF4   */
} dif_kind_t;

static const dif_kind_t k_kinds[AGC_DSKY_DISP_COUNT] = {
    {0u, false, 0},      /* GODSP                                     */
    {0u, true, 0},       /* GODSPR                                    */
    {000040u, false, 0}, /* GODSPRET: BIT6                            */
    {000010u, false, 0}, /* GOFLASH: BIT4                             */
    {000010u, true, 0},  /* GOFLASHR                                  */
    {020000u, false, 0}, /* REGODSP: BIT14                            */
    {020000u, true, 0},  /* REGODSPR                                  */
    {020010u, false, 0}, /* REFLASH: REDOMASK, flash and permit       */
    {020010u, true, 0},  /* REFLASHR (LM)                             */
    {020010u, false, 1}, /* CLEANDSP: REFLASH with A = 0              */
    {000036u, false, 2}, /* GOPERF1: PERFMASK flash,perform,blank R2,R3 */
    {000036u, true, 2},  /* GOPERF1R                                  */
    {000030u, false, 0}, /* GOPERF2: PERF2MSK flash, perform          */
    {000030u, true, 0},  /* GOPERF2R                                  */
    {000014u, false, 3}, /* GOPERF4: PERF4MSK flash, blank R3         */
    {000014u, true, 3},  /* GOPERF4R                                  */
    {000110u, false, 0}, /* PRIODSP: BITS7+4                          */
    {000110u, true, 0},  /* PRIODSPR                                  */
    {040000u, false, 0}, /* GOMARK (GOXDSP): BIT15                    */
    {040000u, true, 0},  /* GOMARKR (GOXDSPR)                         */
    {040010u, false, 0}, /* GOMARKF (GOXDSPF): MARKFMSK               */
    {040010u, true, 0},  /* GOMARKFR (GOXDSPFR)                       */
    {040030u, false, 0}, /* GOMARK2: MPERFMSK mark, perform, flash    */
    {040030u, true, 0},  /* GOMARK2R                                  */
    {040230u, false, 0}, /* GOMARK3: MARK3MSK + decimal noun          */
    {040230u, true, 0},  /* GOMARK3R                                  */
    {040036u, false, 0}, /* GOMARK4: MARK4MSK + blank R2, R3          */
    {040040u, false, 0}, /* EXDSPRET: BIT15+6                         */
    {040010u, false, 1}, /* KLEENEX: GOMARKF with A = 0               */
    {024030u, false, 0}, /* CLOCPLAY: CLOCKCON                        */
};

/* GODSPRS1 / BOTHJOBS: start MAKEPLAY at the user's priority (a VAC job
 * for a flashing display) with the request copied into its MPAC.        */
static agc_status_t dif_start_makeplay(const dif_req_t *rq, agc_prio_t prio)
{
    int slot = -1;

    for (int i = 0; i < DIF_RQ_LEN && slot < 0; i++)
    {
        if (!g_rq[i].used)
        {
            slot = i;
        }
    }

    if (!AGC_ASSERT(slot >= 0) || !AGC_ASSERT(prio >= 1u && prio <= 037u))
    {
        return (AGC_STATUS_FULL);
    }

    agc_inhint();
    agc_status_t st = ((rq->flags & DIF_FLASH) != 0u) ?
                      agc_exec_findvac(prio, AGC_JOB_MAKEPLAY) :
                      agc_exec_novac(prio, AGC_JOB_MAKEPLAY);

    if (st == AGC_STATUS_OK)
    {
        st              = agc_exec_set_new_job_arg((agc_sp_t) (slot + 1));
        g_rq[slot]      = *rq;
        g_rq[slot].used = true;
        g_rq_last       = slot;
    }

    agc_relint();

    return (st);
}

agc_status_t agc_dsky_display(const agc_job_ctx_t *ctx, agc_dsky_disp_t kind,
                              uint16_t vn_or_value, const agc_dsky_cont_t *cont)
{
    if (!AGC_ASSERT(ctx != NULL) ||
        !AGC_ASSERT(kind >= AGC_DSKY_GODSP && kind < AGC_DSKY_DISP_COUNT))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    const dif_kind_t *k  = &k_kinds[kind];
    dif_req_t         rq = {true,
                            vn_or_value,
                            k->flags,
                            {AGC_JOB_NONE, AGC_JOB_NONE, AGC_JOB_NONE}};

    if (cont != NULL)
    {
        rq.cadr = *cont;
    }

    /* Flashing displays need the three astronaut returns; GODSPRET and
     * EXDSPRET return to call CADR +1 (terminate). CLEANDSP and KLEENEX
     * (VN forced to +0) have no return slots in their AGC calling
     * sequence ("TC BANKCALL / CADR CLEANDSP"): they return to the user. */
    if (((k->flags & DIF_FLASH) != 0u && k->src != 1u &&
         !AGC_ASSERT(dif_cont_valid(&rq.cadr))) ||
        ((k->flags & DIF_GODSPRET) != 0u &&
         !AGC_ASSERT(rq.cadr.terminate != AGC_JOB_NONE)))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (k->src == 1u)
    {
        rq.vn = 0; /* CAF ZERO       */

        if (cont == NULL)
        { /* DEVIATION: CLEANDSP / KLEENEX without a continuation run in a
           * MAKEPLAY job of their own (as the R entries do), which ends
           * when JAMTERM (or, after GOSLEEPS, the wake) lets it out of
           * ENDIDLE. The C caller goes on at once and its job is never
           * left asleep; the AGC returns to the user only after that.
           * Exact for callers that end the job next (P40BLNKR, P41BLANK,
           * BLANKDEX); P11 and P61-P67 STARTENT do their next steps
           * before the clean-out instead of after it.                    */
            rq.cadr.terminate = AGC_JOB_ENDOFJOB;
            rq.cadr.proceed   = AGC_JOB_ENDOFJOB;
            rq.cadr.enter     = AGC_JOB_ENDOFJOB;

            return (dif_start_makeplay(&rq, ctx->prio));
        }
    }
    else if (k->src == 2u)
    {
        agc_status_t st = agc_dsky_set_dsptem(
            0, (uint16_t) (vn_or_value & PIN_WORD)); /* NORMTEM1 */

        rq.vn = agc_dsky_vn(1, 25);                  /* V01N25         */

        if (st != AGC_STATUS_OK)
        {
            return (st);
        }
    }
    else if (k->src == 3u)
    {
        agc_erase_set_option1(0,
                              pin_sp_from_ones((uint16_t) (vn_or_value &
                                                           PIN_WORD))); /* PURRS4 */
        rq.vn = agc_dsky_vn(4, 6); /* V04N06         */
    }

    if (k->r)
    {
        return (dif_start_makeplay(&rq, ctx->prio));
    }

    return (dif_makeplay(&rq, ctx->prio)); /* GOFLASH2: SAVELOCS, MAKEPLAY */
}

/* BLANKET (LINUS): OR flag bits into the PLAYTEM4 of the MAKEPLAY job the
 * last R routine started (e.g. blank bits).                             */
agc_status_t agc_dsky_blanket(uint16_t bits)
{
    if (!AGC_ASSERT(g_rq_last >= 0 && g_rq_last < DIF_RQ_LEN) ||
        !AGC_ASSERT(bits <= PIN_WORD))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (!g_rq[g_rq_last].used)
    {
        return (AGC_STATUS_NOT_FOUND); /* already started */
    }

    g_rq[g_rq_last].flags = (uint16_t) (g_rq[g_rq_last].flags | bits);

    return (AGC_STATUS_OK);
}

/* INITDSP: restore the most recent normal user's priority and redo his
 * display request.
 * DEVIATION: the AGC jumps back to the user's CAF VXXNYY / TC BANKCALL
 * (TEMPFLSH -3); here the saved normal request (NVSAVE, EBANKTEM,
 * TEMPFLSH) is re-issued through MAKEPLAY from the current job.        */
agc_status_t agc_dsky_initdsp(void)
{
    if (!AGC_ASSERT(g_restreg >= 1u && g_restreg <= 037u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_exec_prio_change(g_restreg);
    dif_req_t rq = {true, g_cls[DIF_NORMC].nvword, g_cls[DIF_NORMC].flags,
                    g_cls[DIF_NORMC].cadr};

    rq.flags = (uint16_t) (rq.flags & ~DIF_SECOND);

    if (!AGC_ASSERT((rq.flags & (DIF_MARK | DIF_PRIO)) == 0u))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (dif_makeplay(&rq, g_restreg));
}

void agc_dsky_set_nvword1(uint16_t word)
{
    if (!AGC_ASSERT(pin_invariants()))
    {
        return;
    }

    if (!AGC_ASSERT(word <= PIN_WORD))
    {
        return;
    }

    g_nvword1 = word;
}
