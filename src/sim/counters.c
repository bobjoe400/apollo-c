/*
 * counters.c -- unprogrammed counter increments requested by peripherals.
 *
 * A packet on channel 0200 | addr asks for one increment of the counter
 * cell at erasable address addr (yaAGC SocketAPI.c ChannelInput and
 * agc_engine.c UnprogrammedIncrement). The value is the increment type:
 *   0 PINC, 2 MINC          (ones'-complement +1 / -1: PIPAs, RHC)
 *   1, 021 PCDU, 3, 023 MCDU (two's-complement 15-bit +1 / -1: CDUs;
 *                             021/023 are yaAGC's fast-rate variants)
 *   4 DINC                  (diminish toward zero)
 *   5 SHINC, 6 SHANC        (shift in a 0 / a 1: RNRAD, INLINK)
 * The translated program keeps each counter in its owner's storage, so the
 * address is mapped onto the accessor here.
 *
 * DEVIATION: yaAGC paces CDU increments through a FIFO (213 or 13 machine
 * cycles apart); here each increment is applied when the packet arrives.
 * Counters owned by the harness (TIME1-TIME6) and the output counters
 * (GYROCMD, CDUxCMD, THRUST, ALTM, OUTLINK) take no input increments.
 */
#include <stddef.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"
#include "internal.h"

#define CTR_CDUX   032u
#define CTR_CDUY   033u
#define CTR_CDUZ   034u
#define CTR_OPTY   035u /* CDUT: CSM optics trunnion / LM RR trunnion */
#define CTR_OPTX   036u /* CDUS: CSM optics shaft / LM RR shaft       */
#define CTR_PIPAX  037u
#define CTR_PIPAY  040u
#define CTR_PIPAZ  041u
#define CTR_RHC_Q  042u /* LM Q-RHCCTR (pitch)                        */
#define CTR_RHC_P  043u /* LM P-RHCCTR                                */
#define CTR_RHC_R  044u /* LM R-RHCCTR                                */
#define CTR_INLINK 045u
#define CTR_RNRAD  046u

#define INC_PINC  0u
#define INC_PCDU  1u
#define INC_MINC  2u
#define INC_MCDU  3u
#define INC_DINC  4u
#define INC_SHINC 5u
#define INC_SHANC 6u
#define INC_FAST  020u /* yaAGC 021 / 023: fast CDU rate          */

/* +1 / -1 / toward zero for a signed SP counter; PINC past POSMAX wraps
 * to +0 (the overflow the hardware would turn into a fail bit).        */
static agc_sp_t sp_step(agc_sp_t v, uint16_t type)
{
    int32_t n = (int32_t) v;

    if (type == INC_PINC || type == INC_PCDU)
    {
        n = (n >= (int32_t) AGC_SP_MAX) ? 0 : n + 1;
    }
    else if (type == INC_MINC || type == INC_MCDU)
    {
        n = (n <= (int32_t) AGC_SP_MIN) ? 0 : n - 1;
    }
    else if (type == INC_DINC)
    {
        n = (n > 0) ? n - 1 : ((n < 0) ? n + 1 : 0);
    }

    return ((agc_sp_t) n);
}

/* +1 / -1 on a two's-complement 15-bit CDU word.                       */
static uint16_t cdu_step(uint16_t v, uint16_t type)
{
    bool     up = (type == INC_PINC || type == INC_PCDU);
    bool     dn = (type == INC_MINC || type == INC_MCDU);
    uint32_t n  = up ? (uint32_t) v + 1u : (dn ? (uint32_t) v + SIM_WORD_MASK : (uint32_t) v);

    return ((uint16_t) (n & SIM_WORD_MASK));
}

/* SHINC / SHANC: shift left, bring in 0 / 1 (radar and uplink data).    */
static uint16_t shift_in(uint16_t v, uint16_t type)
{
    uint16_t n = (uint16_t) (((uint32_t) v << 1) & SIM_WORD_MASK);

    if (type == INC_SHANC)
    {
        n = (uint16_t) (n | 1u);
    }

    return (n);
}

static agc_status_t cdu_increment(uint16_t addr, uint16_t type)
{
    switch (addr)
    {
        case CTR_CDUX:
            agc_erase_set_cdux(cdu_step(agc_erase_cdux(), type));
            break;
        case CTR_CDUY:
            agc_erase_set_cduy(cdu_step(agc_erase_cduy(), type));
            break;
        case CTR_CDUZ:
            agc_erase_set_cduz(cdu_step(agc_erase_cduz(), type));
            break;
        case CTR_OPTY:
            agc_erase_set_cdut(cdu_step(agc_erase_cdut(), type));
            break;
        default:
            agc_erase_set_cdus(cdu_step(agc_erase_cdus(), type));
            break;
    }

    if (!AGC_ASSERT(agc_erase_cdux() <= SIM_WORD_MASK))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

static agc_status_t sp_increment(uint16_t addr, uint16_t type)
{
    switch (addr)
    {
        case CTR_PIPAX:
            agc_erase_set_pipax(sp_step(agc_erase_pipax(), type));
            break;
        case CTR_PIPAY:
            agc_erase_set_pipay(sp_step(agc_erase_pipay(), type));
            break;
        case CTR_PIPAZ:
            agc_erase_set_pipaz(sp_step(agc_erase_pipaz(), type));
            break;
        case CTR_RHC_Q:
            agc_erase_set_q_rhcctr(sp_step(agc_erase_q_rhcctr(), type));
            break;
        case CTR_RHC_P:
            agc_erase_set_p_rhcctr(sp_step(agc_erase_p_rhcctr(), type));
            break;
        default:
            agc_erase_set_r_rhcctr(sp_step(agc_erase_r_rhcctr(), type));
            break;
    }

    if (!AGC_ASSERT(type <= INC_SHANC))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

agc_status_t sim_counter_increment(uint16_t addr, uint16_t type)
{
    uint16_t t = (type == (INC_FAST | INC_PCDU) || type == (INC_FAST | INC_MCDU)) ? (uint16_t) (type & ~INC_FAST) : type;

    if (!AGC_ASSERT(addr < SIM_CH_COUNTER) || t > INC_SHANC)
    {
        return (AGC_STATUS_BAD_ARG);
    }

    bool shift = (t == INC_SHINC || t == INC_SHANC);

    if (addr >= CTR_CDUX && addr <= CTR_OPTX && !shift)
    {
        return (cdu_increment(addr, t));
    }

    if (addr >= CTR_PIPAX && addr <= CTR_RHC_R && !shift)
    {
        return (sp_increment(addr, t));
    }

    if (addr == CTR_RNRAD && shift)
    {
        agc_erase_set_rnrad(shift_in(agc_erase_rnrad(), t));

        return (AGC_STATUS_OK);
    }

    if (addr == CTR_INLINK && shift)
    {
        agc_erase_set_inlink(shift_in(agc_erase_inlink(), t));

        return (AGC_STATUS_OK);
    }

    return (AGC_STATUS_NOT_FOUND); /* timers and output counters      */
}
