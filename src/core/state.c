/*
 * agc_state.c -- vehicle selection, TIME1/TIME2, flagwords, state vectors.
 *
 * The AGC keeps FLAGWRD0.. in consecutive erasable words (STATE + n). Here
 * they are an array of 15-bit words indexed by n; a flag id is
 * (n << 4) | bit with bit = 1..15 (bit 1 = least significant), see
 * agc_flags_common.h. The other shared erasables are in agc_erasable.c.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_common.h"
#include "agc/core/state.h"

#define AGC_WORD15_MASK 0x7FFFu
#define AGC_TIME1_BITS  14 /* TIME1 holds the low 14 bits of the count */
#define AGC_TIME_FIELD  0x3FFFu

static agc_vehicle_t g_vehicle = AGC_VEHICLE_CSM;
static uint32_t      g_time_cs;                 /* C(TIME2,TIME1), 28 bits */
static uint16_t      g_flagword[AGC_FLAGWORDS]; /* FLAGWRD0..FLAGWRD13     */

/* ---- vehicle ----------------------------------------------------------- */

static bool vehicle_ok(void)
{
    return (g_vehicle == AGC_VEHICLE_CSM) || (g_vehicle == AGC_VEHICLE_LM);
}

void agc_state_init(agc_vehicle_t v)
{
    if (!AGC_ASSERT((v == AGC_VEHICLE_CSM) || (v == AGC_VEHICLE_LM)))
    {
        return;
    }

    g_vehicle = v;
    g_time_cs = 0;

    for (int i = 0; i < AGC_FLAGWORDS; i++)
    {
        g_flagword[i] = 0;
    }

    agc_erase_reset();
    agc_chan_reset();

    if (!AGC_ASSERT(vehicle_ok()))
    {
        return;
    }

    if (!AGC_ASSERT((g_time_cs == 0u) && (g_flagword[0] == 0u)))
    {
        return;
    }
}

agc_vehicle_t agc_state_vehicle(void)
{
    if (!AGC_ASSERT(vehicle_ok()))
    {
        return AGC_VEHICLE_CSM;
    }

    return g_vehicle;
}

/* ---- time (TIME2,TIME1 centisecond counter) ---------------------------- */

uint32_t agc_time_now_cs(void)
{
    if (!AGC_ASSERT(g_time_cs < AGC_TIME_MODULUS_CS))
    {
        return 0;
    }

    if (!AGC_ASSERT(vehicle_ok()))
    {
        return 0;
    }

    return g_time_cs;
}

void agc_time_set_cs(uint32_t t_cs)
{
    if (!AGC_ASSERT(t_cs < AGC_TIME_MODULUS_CS))
    {
        return;
    }

    g_time_cs = t_cs;

    if (!AGC_ASSERT(g_time_cs == t_cs))
    {
        return;
    }
}

void agc_time_advance_cs(uint16_t dt_cs)
{
    if (!AGC_ASSERT(g_time_cs < AGC_TIME_MODULUS_CS))
    {
        return;
    }

    /* TIME2 overflow wraps the whole 28-bit count to zero. */
    g_time_cs = (g_time_cs + dt_cs) & (AGC_TIME_MODULUS_CS - 1u);

    if (!AGC_ASSERT(g_time_cs < AGC_TIME_MODULUS_CS))
    {
        g_time_cs = 0;
    }
}

agc_sp_t agc_erase_time1(void)
{
    if (!AGC_ASSERT(g_time_cs < AGC_TIME_MODULUS_CS))
    {
        return 0;
    }

    if (!AGC_ASSERT(vehicle_ok()))
    {
        return 0;
    }

    return (agc_sp_t) (g_time_cs & AGC_TIME_FIELD);
}

agc_sp_t agc_erase_time2(void)
{
    if (!AGC_ASSERT(g_time_cs < AGC_TIME_MODULUS_CS))
    {
        return 0;
    }

    if (!AGC_ASSERT(vehicle_ok()))
    {
        return 0;
    }

    return (agc_sp_t) ((g_time_cs >> AGC_TIME1_BITS) & AGC_TIME_FIELD);
}

/* ---- flags ------------------------------------------------------------- */

static int words_for_vehicle(void)
{
    if (!AGC_ASSERT(vehicle_ok()))
    {
        return AGC_FLAGWORDS_CSM;
    }

    return (g_vehicle == AGC_VEHICLE_LM) ? AGC_FLAGWORDS_LM : AGC_FLAGWORDS_CSM;
}

bool agc_flag_valid(agc_flag_id_t f)
{
    const int word = (int) (f >> 4);
    const int bit  = (int) (f & 0xFu);

    if (!AGC_ASSERT(f <= 0xFFu))
    {
        return false;
    }

    if (!AGC_ASSERT(bit >= 0))
    {
        return false;
    }

    return (word < words_for_vehicle()) && (bit >= 1) && (bit <= 15);
}

int agc_flag_word(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return -1;
    }

    if (!AGC_ASSERT(((int) (f >> 4)) < AGC_FLAGWORDS))
    {
        return -1;
    }

    return (int) (f >> 4);
}

int agc_flag_bit(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return -1;
    }

    if (!AGC_ASSERT((f & 0xFu) != 0u))
    {
        return -1;
    }

    return (int) (f & 0xFu);
}

uint16_t agc_flag_mask(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return 0;
    }

    if (!AGC_ASSERT((f & 0xFu) <= 15u))
    {
        return 0;
    }

    return (uint16_t) (1u << ((f & 0xFu) - 1u));
}

bool agc_flag_test(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return false;
    }

    if (!AGC_ASSERT(agc_flag_mask(f) != 0u))
    {
        return false;
    }

    return (g_flagword[f >> 4] & agc_flag_mask(f)) != 0u;
}

void agc_flag_set(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return;
    }

    g_flagword[f >> 4] = (uint16_t) (g_flagword[f >> 4] | agc_flag_mask(f));

    if (!AGC_ASSERT(agc_flag_test(f)))
    {
        return;
    }
}

void agc_flag_clear(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return;
    }

    g_flagword[f >> 4] = (uint16_t) (g_flagword[f >> 4] & ~agc_flag_mask(f));

    if (!AGC_ASSERT(!agc_flag_test(f)))
    {
        return;
    }
}

void agc_flag_invert(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return;
    }

    const bool before = agc_flag_test(f);

    g_flagword[f >> 4] = (uint16_t) (g_flagword[f >> 4] ^ agc_flag_mask(f));

    if (!AGC_ASSERT(agc_flag_test(f) != before))
    {
        return;
    }
}

/* SET or CLEAR by a computed condition (the AGC tests and branches to one
 * of the two instructions; in C the condition is the argument).        */
void agc_flag_put(agc_flag_id_t f, bool on)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return;
    }

    if (on)
    {
        agc_flag_set(f);
    }
    else
    {
        agc_flag_clear(f);
    }

    if (!AGC_ASSERT(agc_flag_test(f) == on))
    {
        return;
    }
}

uint16_t agc_flagword_get(int word)
{
    if (!AGC_ASSERT((word >= 0) && (word < words_for_vehicle())))
    {
        return 0;
    }

    if (!AGC_ASSERT(g_flagword[word] <= AGC_WORD15_MASK))
    {
        return 0;
    }

    return g_flagword[word];
}

void agc_flagword_set(int word, uint16_t value)
{
    if (!AGC_ASSERT((word >= 0) && (word < words_for_vehicle())))
    {
        return;
    }

    if (!AGC_ASSERT(value <= AGC_WORD15_MASK))
    {
        return;
    }

    g_flagword[word] = value;

    if (!AGC_ASSERT(g_flagword[word] == value))
    {
        return;
    }
}

/* ---- state vectors ------------------------------------------------------ */

static agc_flag_id_t other_moon_flag(void)
{
    /* The flag describing the OTHER vehicle's permanent state vector. */
    if (!AGC_ASSERT(vehicle_ok()))
    {
        return (agc_flag_id_t) AGC_FLAG_LMOONFLG;
    }

    const agc_flag_id_t f = (g_vehicle == AGC_VEHICLE_CSM) ?
                            (agc_flag_id_t) AGC_FLAG_LMOONFLG :
                            (agc_flag_id_t) AGC_FLAG_CMOONFLG;

    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return (agc_flag_id_t) AGC_FLAG_LMOONFLG;
    }

    return f;
}

agc_status_t agc_sv_get(agc_vehicle_sv_t which, agc_state_vector_t *out)
{
    if (!AGC_ASSERT(out != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT((which == AGC_SV_THIS) || (which == AGC_SV_OTHER)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (which == AGC_SV_THIS)
    {
        out->r    = agc_erase_rn();
        out->v    = agc_erase_vn();
        out->t    = agc_erase_piptime();
        out->moon = agc_flag_test(AGC_FLAG_MOONFLAG);
    }
    else if (g_vehicle == AGC_VEHICLE_CSM)
    {
        out->r    = agc_erase_rrectlem();
        out->v    = agc_erase_vrectlem();
        out->t    = agc_erase_tetlem();
        out->moon = agc_flag_test(other_moon_flag());
    }
    else
    {
        out->r    = agc_erase_rrectcsm();
        out->v    = agc_erase_vrectcsm();
        out->t    = agc_erase_tetcsm();
        out->moon = agc_flag_test(other_moon_flag());
    }

    if (!AGC_ASSERT(vehicle_ok()))
    {
        return AGC_STATUS_BAD_ARG;
    }

    return AGC_STATUS_OK;
}

static agc_status_t set_moon(agc_flag_id_t f, bool moon)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT((f >> 4) < AGC_FLAGWORDS))
    {
        return AGC_STATUS_BAD_ARG;
    }

    agc_flag_put(f, moon);

    return AGC_STATUS_OK;
}

agc_status_t agc_sv_set(agc_vehicle_sv_t which, const agc_state_vector_t *in)
{
    if (!AGC_ASSERT(in != NULL))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (!AGC_ASSERT((which == AGC_SV_THIS) || (which == AGC_SV_OTHER)))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (which == AGC_SV_THIS)
    {
        agc_erase_set_rn(in->r);
        agc_erase_set_vn(in->v);
        agc_erase_set_piptime(in->t);

        return set_moon(AGC_FLAG_MOONFLAG, in->moon);
    }

    if (!AGC_ASSERT(vehicle_ok()))
    {
        return AGC_STATUS_BAD_ARG;
    }

    if (g_vehicle == AGC_VEHICLE_CSM)
    {
        agc_erase_set_rrectlem(in->r);
        agc_erase_set_vrectlem(in->v);
        agc_erase_set_tetlem(in->t);
    }
    else
    {
        agc_erase_set_rrectcsm(in->r);
        agc_erase_set_vrectcsm(in->v);
        agc_erase_set_tetcsm(in->t);
    }

    return set_moon(other_moon_flag(), in->moon);
}
