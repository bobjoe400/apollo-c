/**********************************************************************
 * FILE NAME: counters.c
 *
 * PURPOSE:
 *   Unprogrammed counter increments requested by peripherals: a packet
 *   on channel 0200 | addr asks for one increment of the counter cell at
 *   erasable address addr, the value being the increment type.
 *
 * AGC SOURCE:
 *   No AGC routine: virtualagc yaAGC/agc_engine.c UnprogrammedIncrement
 *   and CounterPINC, CounterMINC, CounterPCDU, CounterMCDU, CounterDINC,
 *   CounterSHINC, CounterSHANC (E-2052 1.4.8).
 *
 * EXTERNAL VARIABLES:
 *   Name            Type      I/O  Scale   Description
 *   --------------  --------  ---  ------  ---------------------------
 *   CDUX-CDUZ, CDUT, CDUS  uint16_t I/O  2^-15 rev  CDU counters (032-036)
 *   PIPAX-PIPAZ     agc_sp_t  I/O  pulses  PIPA counters (037-041)
 *   Q/P/R-RHCCTR    agc_sp_t  I/O  counts  LM RHC counters (042-044)
 *   INLINK, RNRAD   uint16_t  I/O  word    uplink / radar (045, 046)
 *   CDUTCMD, CDUSCMD, THRUST agc_sp_t I/O pulses  output counters
 *                                          053-055 (DINC only)
 *
 * EXTERNAL REFERENCES:
 *   Name                  Description
 *   --------------------  ------------------------------------------
 *   sim_socket_send       POUT / MOUT / ZOUT of a DINC (socket.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   sim_counter_increment returns AGC_STATUS_BAD_ARG for an increment
 *   type yaAGC does not know, AGC_STATUS_NOT_FOUND for a counter the
 *   harness does not map (timers, GYROCMD, CDUxCMD, ALTM, OUTLINK) or a
 *   type that does not apply to it.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The translated program keeps each counter in its owner's storage,
 *   so the address is mapped onto the accessor. SP counters hold no -0;
 *   PINC, MINC and DINC give the same results from +0 and -0.
 *
 * NOTES:
 *   Types: 0 PINC, 2 MINC (ones' complement +1 / -1, overflow at
 *   +-37777 to +-0); 1, 021 PCDU, 3, 023 MCDU (two's complement 15-bit
 *   +1 / -1; 021/023 are yaAGC's fast-rate variants); 4 DINC (toward
 *   zero; at +-0 nothing changes and ZOUT is sent, else POUT / MOUT, as
 *   packets 0200 | addr = 017 / 015 / 016); 5 SHINC, 6 SHANC (shift left
 *   in 14 bits, bring in 0 / 1).
 *   CDUX-CDUZ PCDU/MCDU go through a FIFO per CDU, as in yaAGC
 *   (PushCduFifo / ServiceCduFifo): runs of one type (slow or fast, up
 *   or down) in arrival order, at most 128 runs (a count that would need
 *   a 129th is dropped), applied at the CDU's own rate, 400 counts a
 *   second slow (types 1, 3) and 6400 fast (021, 023). A peripheral may
 *   send counts faster than that (the LM Simulator sends a coarse-align
 *   gimbal step at once); the AGC then sees its CDUs lag the gimbals, as
 *   COARS's CHKCORS does under yaAGC (closed-loop-fixes S-L2).
 *   DEVIATION (pacing grain): the FIFO is served once per centisecond
 *   (sim_counters_tick) with a budget of 64 fast counts (or 4 slow, a
 *   slow count costing 16), where yaAGC serves it every 213 (13) machine
 *   cycles; the rate is the same, a count lands up to 10 ms later.
 *   Counters owned by the harness (TIME1-TIME6) and the drives it
 *   empties itself (GYROCMD, CDUxCMD, channels.c) take no increments.
 *
 * ALGORITHM:
 *   word = the counter as a 15-bit word; step it as yaAGC does; store.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "internal.h"

#define CTR_CDUX     032u    /* CDUX                                   */
#define CTR_CDUS     036u    /* CDUS (CSM optics shaft / LM RR shaft)  */
#define CTR_CDUT     035u    /* CDUT (CSM optics trunnion / LM RR trunnion) */
#define CTR_PIPAX    037u    /* PIPAX                                  */
#define CTR_PIPAY    040u    /* PIPAY                                  */
#define CTR_PIPAZ    041u    /* PIPAZ                                  */
#define CTR_RHC_Q    042u    /* LM Q-RHCCTR (pitch)                    */
#define CTR_RHC_P    043u    /* LM P-RHCCTR (yaw)                      */
#define CTR_RHC_R    044u    /* LM R-RHCCTR (roll)                     */
#define CTR_INLINK   045u    /* INLINK                                 */
#define CTR_RNRAD    046u    /* RNRAD                                  */
#define CTR_CDUTCMD  053u    /* CDUTCMD (CSM TVC yaw)                  */
#define CTR_CDUSCMD  054u    /* CDUSCMD (CSM TVC pitch)                */
#define CTR_THRUST   055u    /* LM THRUST                              */
#define CTR_ADDR_MAX 0177u   /* counter addresses: 7 bits              */
#define INC_PINC     0u      /* ones'-complement +1                    */
#define INC_PCDU     1u      /* two's-complement +1                    */
#define INC_MINC     2u      /* ones'-complement -1                    */
#define INC_MCDU     3u      /* two's-complement -1                    */
#define INC_DINC     4u      /* diminish toward zero                   */
#define INC_SHINC    5u      /* shift in a 0                           */
#define INC_SHANC    6u      /* shift in a 1                           */
#define INC_FAST     020u    /* yaAGC 021 / 023: fast CDU rate         */
#define OUT_POUT     015u    /* DINC output: plus pulse                */
#define OUT_MOUT     016u    /* DINC output: minus pulse               */
#define OUT_ZOUT     017u    /* DINC output: zero                      */
#define W_POSMAX     037777u /* +37777                                 */
#define W_NEGMAX     040000u /* -37777                                 */
#define W_NEG0       077777u /* -0                                     */
#define W_SIGN       040000u /* sign bit                               */
#define W_SHIFT_MASK 037777u /* SHINC / SHANC keep 14 bits             */
#define CDU_FIFOS    3       /* CDUX-CDUZ (yaAGC NUM_CDU_FIFOS)        */
#define CDU_RUNS     128     /* yaAGC MAX_CDU_FIFO_ENTRIES             */
#define CDU_BUDGET   64u     /* fast counts per cs: 6400 a second      */
#define CDU_COST_SLW 16u     /* a slow count: 400 a second = 64 / 16   */

/* One FIFO entry: a run of counts of one type (yaAGC Counts[i]).          */
typedef struct cdu_run
{
    uint16_t type;  /* INC_PCDU / INC_MCDU, | INC_FAST for the fast rate */
    uint32_t count; /* counts still to apply                           */
} cdu_run_t;

/* A CDU's FIFO (yaAGC CduFifo_t): a ring of runs.                         */
typedef struct cdu_fifo
{
    int       head;          /* index of the run being applied */
    int       size;          /* runs queued                    */
    cdu_run_t run[CDU_RUNS]; /* the runs, oldest at head       */
} cdu_fifo_t;

static cdu_fifo_t g_cdu_fifo[CDU_FIFOS]; /* CDUX, CDUY, CDUZ */

/**********************************************************************
 * AGC: none -- yaAGC CounterDINC: the step and the output it emits.
 *
 * ARGUMENTS:
 *   w    uint16_t    I  counter, 15-bit word
 *   out  uint16_t *  O  OUT_POUT, OUT_MOUT or OUT_ZOUT
 *
 * RETURNS: uint16_t -- the counter after the DINC
 **********************************************************************/
static uint16_t word_dinc(uint16_t w, uint16_t *out)
{
    uint16_t n = w; /* +-0: unchanged */

    if (w == 0u || w == W_NEG0)
    {
        *out = OUT_ZOUT;
    }
    else if ((w & W_SIGN) != 0u)
    {
        *out = OUT_MOUT;
        n    = (uint16_t) (w + 1u); /* toward -0 */
    }
    else if (w == 1u)
    {
        *out = OUT_POUT;
        n    = W_NEG0; /* +1 - 1 = -0 in the adder */
    }
    else
    {
        *out = OUT_POUT;
        n    = (uint16_t) (w - 1u);
    }

    return (n);
}

/**********************************************************************
 * AGC: none -- one increment of a 15-bit counter word as yaAGC's
 * Counter* functions do it (DINC: word_dinc).
 *
 * ARGUMENTS:
 *   w     uint16_t  I  counter, 15-bit word
 *   type  uint16_t  I  INC_PINC .. INC_SHANC, not INC_DINC
 *
 * RETURNS: uint16_t -- the counter after the increment
 **********************************************************************/
static uint16_t word_step(uint16_t w, uint16_t type)
{
    uint32_t n = w; /* result before masking */

    switch (type)
    {
        case INC_PINC:
            n = ((uint32_t) w + 1u) & SIM_WORD_MASK;
            n = (w == W_POSMAX) ? 0u : n;                  /* overflow to +0 */
            n = (n == 0u && w != W_POSMAX) ? 1u : n;       /* -0 + 1 = +1 */
            break;
        case INC_MINC:
            n = ((uint32_t) w + SIM_WORD_MASK) & SIM_WORD_MASK;
            n = (w == W_NEGMAX) ? W_NEG0 : n;              /* overflow to -0 */
            n = (w == 0u) ? W_NEG0 - 1u : n;               /* +0 - 1 = -1 */
            break;
        case INC_PCDU:
            n = (uint32_t) w + 1u;
            break;
        case INC_MCDU:
            n = (uint32_t) w + SIM_WORD_MASK;
            break;
        case INC_SHINC:
            n = ((uint32_t) w << 1) & W_SHIFT_MASK;
            break;
        default:
            n = (((uint32_t) w << 1) + 1u) & W_SHIFT_MASK; /* SHANC */
            break;
    }

    return ((uint16_t) (n & SIM_WORD_MASK));
}

/**********************************************************************
 * AGC: none -- an SP counter value as its 15-bit ones'-complement word.
 *
 * ARGUMENTS:
 *   v  agc_sp_t  I  value, +-16383
 *
 * RETURNS: uint16_t -- the word (0 as +0)
 **********************************************************************/
static uint16_t word_of_sp(agc_sp_t v)
{
    uint16_t w = (v < 0) ? (uint16_t) (~(uint16_t) (-(int32_t) v) & SIM_WORD_MASK) : (uint16_t) v; /* negative: complement */

    return (w);
}

/**********************************************************************
 * AGC: none -- a 15-bit ones'-complement word as an SP value.
 *
 * ARGUMENTS:
 *   w  uint16_t  I  word
 *
 * RETURNS: agc_sp_t -- value (-0 -> 0)
 **********************************************************************/
static agc_sp_t sp_of_word(uint16_t w)
{
    agc_sp_t v = ((w & W_SIGN) != 0u) ? (agc_sp_t) -(int32_t) ((uint16_t) ~w & W_POSMAX) : (agc_sp_t) w; /* B0 count; negative: complement */

    return (v);
}

/**********************************************************************
 * AGC: none -- the counter at addr as a 15-bit word.
 *
 * ARGUMENTS:
 *   addr  uint16_t    I  counter address
 *   w     uint16_t *  O  the word
 *
 * RETURNS: bool -- false for a counter the harness does not map
 **********************************************************************/
static bool counter_read(uint16_t addr, uint16_t *w)
{
    bool found = true; /* addr is mapped */

    switch (addr)
    {
        case CTR_CDUX:
            *w    = agc_erase_cdux();
            break;
        case CTR_CDUX + 1u:
            *w    = agc_erase_cduy();
            break;
        case CTR_CDUX + 2u:
            *w    = agc_erase_cduz();
            break;
        case CTR_CDUT:
            *w    = agc_erase_cdut();
            break;
        case CTR_CDUS:
            *w    = agc_erase_cdus();
            break;
        case CTR_PIPAX:
            *w    = word_of_sp(agc_erase_pipax());
            break;
        case CTR_PIPAY:
            *w    = word_of_sp(agc_erase_pipay());
            break;
        case CTR_PIPAZ:
            *w    = word_of_sp(agc_erase_pipaz());
            break;
        case CTR_RHC_Q:
            *w    = word_of_sp(agc_erase_q_rhcctr());
            break;
        case CTR_RHC_P:
            *w    = word_of_sp(agc_erase_p_rhcctr());
            break;
        case CTR_RHC_R:
            *w    = word_of_sp(agc_erase_r_rhcctr());
            break;
        case CTR_INLINK:
            *w    = agc_erase_inlink();
            break;
        case CTR_RNRAD:
            *w    = agc_erase_rnrad();
            break;
        case CTR_CDUTCMD:
        case CTR_CDUSCMD:
            *w    = word_of_sp(agc_erase_cdutcmd((int) (addr - CTR_CDUTCMD)));
            break;
        case CTR_THRUST:
            *w    = word_of_sp(agc_erase_thrust());
            break;
        default:
            found = false;
            break;
    } /* switch (addr) */

    return (found);
} /* counter_read */

/**********************************************************************
 * AGC: none -- store a counter word read by counter_read (same addr).
 *
 * ARGUMENTS:
 *   addr  uint16_t  I  counter address, mapped
 *   w     uint16_t  I  the word
 **********************************************************************/
static void counter_write(uint16_t addr, uint16_t w)
{
    switch (addr)
    {
        case CTR_CDUX:
            agc_erase_set_cdux(w);
            break;
        case CTR_CDUX + 1u:
            agc_erase_set_cduy(w);
            break;
        case CTR_CDUX + 2u:
            agc_erase_set_cduz(w);
            break;
        case CTR_CDUT:
            agc_erase_set_cdut(w);
            break;
        case CTR_CDUS:
            agc_erase_set_cdus(w);
            break;
        case CTR_PIPAX:
            agc_erase_set_pipax(sp_of_word(w));
            break;
        case CTR_PIPAY:
            agc_erase_set_pipay(sp_of_word(w));
            break;
        case CTR_PIPAZ:
            agc_erase_set_pipaz(sp_of_word(w));
            break;
        case CTR_RHC_Q:
            agc_erase_set_q_rhcctr(sp_of_word(w));
            break;
        case CTR_RHC_P:
            agc_erase_set_p_rhcctr(sp_of_word(w));
            break;
        case CTR_RHC_R:
            agc_erase_set_r_rhcctr(sp_of_word(w));
            break;
        case CTR_INLINK:
            agc_erase_set_inlink(w);
            break;
        case CTR_RNRAD:
            agc_erase_set_rnrad(w);
            break;
        case CTR_CDUTCMD:
        case CTR_CDUSCMD:
            agc_erase_set_cdutcmd((int) (addr - CTR_CDUTCMD), sp_of_word(w));
            break;
        default:
            agc_erase_set_thrust(sp_of_word(w)); /* CTR_THRUST */
            break;
    } /* switch (addr) */
} /* counter_write */

/**********************************************************************
 * AGC: none -- yaAGC PushCduFifo: queue one PCDU / MCDU of a CDU; a
 * count of the type of the last run extends it, another type opens a
 * run, and with all CDU_RUNS runs in use the count is dropped.
 *
 * ARGUMENTS:
 *   cdu   int       I  0 CDUX, 1 CDUY, 2 CDUZ
 *   type  uint16_t  I  1, 3, 021 or 023
 **********************************************************************/
static void cdu_push(int cdu, uint16_t type)
{
    cdu_fifo_t *f    = &g_cdu_fifo[cdu];                              /* this CDU's FIFO */
    int         last = (f->head + f->size - 1 + CDU_RUNS) % CDU_RUNS; /* newest run */

    if (f->size > 0 && f->run[last].type == type)
    {
        f->run[last].count++;
    }
    else if (f->size < CDU_RUNS)
    {
        last         = (f->head + f->size) % CDU_RUNS;
        f->run[last] = (cdu_run_t) {type, 1u};
        f->size++;
    }
    else
    {
        /* FIFO full: yaAGC drops the count */
    }
}

/**********************************************************************
 * AGC: none -- yaAGC ServiceCduFifo for one centisecond: apply the
 * queued counts of each CDU at 6400 a second (fast) or 400 (slow).
 *
 * ARGUMENTS: none
 **********************************************************************/
void sim_counters_tick(void)
{
    for (int cdu = 0; cdu < CDU_FIFOS; cdu++)
    {
        cdu_fifo_t *f      = &g_cdu_fifo[cdu];                       /* this CDU's FIFO */
        uint32_t    budget = CDU_BUDGET;                             /* fast counts left in this cs */
        uint16_t    addr   = (uint16_t) (CTR_CDUX + (uint16_t) cdu); /* CDUX + cdu */

        for (uint32_t i = 0; i < CDU_BUDGET && f->size > 0; i++)
        {
            cdu_run_t *r    = &f->run[f->head];                                 /* oldest run */
            uint32_t   cost = ((r->type & INC_FAST) != 0u) ? 1u : CDU_COST_SLW; /* fast or slow count */
            uint16_t   w    = 0;                                                /* the CDU word */

            if (budget < cost || !counter_read(addr, &w))
            {
                break;
            }

            budget -= cost;
            counter_write(addr, word_step(w, (uint16_t) (r->type & ~INC_FAST)));
            r->count--;

            if (r->count == 0u)
            {
                f->head = (f->head + 1) % CDU_RUNS;
                f->size--;
            }
        }
    }
}

/**********************************************************************
 * AGC: none -- yaAGC UnprogrammedIncrement: one increment of type
 * `type` of the counter at `addr`. The output counters 053-055 take
 * DINC only (the drive pulses a peripheral counts out of them).
 *
 * ARGUMENTS:
 *   addr  uint16_t  I  counter address, 0..0177
 *   type  uint16_t  I  increment type 0-6, 021, 023
 *
 * RETURNS: agc_status_t -- OK; NOT_FOUND for a counter not mapped (or a
 *          non-DINC on an output counter); BAD_ARG for an unknown type
 **********************************************************************/
agc_status_t sim_counter_increment(uint16_t addr, uint16_t type)
{
    agc_status_t st     = AGC_STATUS_OK;                                                                                           /* outcome */
    uint16_t     t      = (type == (INC_FAST | INC_PCDU) || type == (INC_FAST | INC_MCDU)) ? (uint16_t) (type & ~INC_FAST) : type; /* fast = slow here */
    uint16_t     w      = 0;                                                                                                       /* counter word */
    bool         output = addr >= CTR_CDUTCMD;                                                                                     /* 053-055 */

    if (!AGC_ASSERT(addr <= CTR_ADDR_MAX && t <= INC_SHANC))
    {
        st = AGC_STATUS_BAD_ARG;
    }
    else if (!counter_read(addr, &w) || (output && t != INC_DINC))
    {
        st = AGC_STATUS_NOT_FOUND; /* timers, drives the harness empties, unmapped */
    }
    else if (addr >= CTR_CDUX && addr < CTR_CDUX + CDU_FIFOS && (t == INC_PCDU || t == INC_MCDU))
    {
        cdu_push((int) (addr - CTR_CDUX), type);
    }
    else if (t == INC_DINC)
    {
        uint16_t out = OUT_ZOUT; /* POUT / MOUT / ZOUT */

        counter_write(addr, word_dinc(w, &out));
        sim_socket_send((uint16_t) (SIM_CH_COUNTER | addr), out);
    }
    else
    {
        counter_write(addr, word_step(w, t));
    }

    return (st);
}
