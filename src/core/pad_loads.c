/*
 * pad_loads.c -- pad-loaded erasables (B(n)PL / I(n)PL) read by several lanes.
 *
 * Accessors in the style of erasable.c: one static object per AGC erasable,
 * a range check on every store, scale factors in include/agc/core/state.h.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"

#define EW15_MAX 077777u /* a raw 15-bit AGC word */

/* AOTAZ -1+n, AOTEL -1+n (LM, B(6)PL): AOT detent n = 1..6 azimuth and
 * elevation, CDU words (2^15 = 1 revolution).                             */
static uint16_t g_aotaz[6];
static uint16_t g_aotel[6];

/* CDUCHKWD (CSM, B(1)PL): CDU check delay for R60 / IMU checks, cs.       */
static agc_sp_t g_cduchkwd;

/* RTED1 (CSM, I(2)PL): VGAMMA polynomial coefficient D1 of P37, B-3.      */
static agc_dp_t g_rted1;

/* P37RANGE (= R61CNTR +1, CSM I(1)PL): entry range override, revs.       */
static agc_sp_t g_p37range;

/* TLAND (= W, LM I(2)): nominal time of landing, cs 2^28.                 */
static agc_dp_t g_tland;

void agc_erase_reset_pad_loads(void)
{
    for (int i = 0; i < 6; i++)
    {
        g_aotaz[i] = 0;
        g_aotel[i] = 0;
    }

    g_cduchkwd = 0;
    g_rted1    = 0;
    g_p37range = 0;
    g_tland    = 0;
}

uint16_t agc_erase_aotaz(int index)
{
    if (!AGC_ASSERT(index >= 1 && index <= 6))
    {
        return (0);
    }

    if (!AGC_ASSERT(g_aotaz[index - 1] <= EW15_MAX))
    {
        return (0);
    }

    return (g_aotaz[index - 1]);
}

void agc_erase_set_aotaz(int index, uint16_t v)
{
    if (!AGC_ASSERT(index >= 1 && index <= 6))
    {
        return;
    }

    if (!AGC_ASSERT(v <= EW15_MAX))
    {
        return;
    }

    g_aotaz[index - 1] = v;
}

uint16_t agc_erase_aotel(int index)
{
    if (!AGC_ASSERT(index >= 1 && index <= 6))
    {
        return (0);
    }

    if (!AGC_ASSERT(g_aotel[index - 1] <= EW15_MAX))
    {
        return (0);
    }

    return (g_aotel[index - 1]);
}

void agc_erase_set_aotel(int index, uint16_t v)
{
    if (!AGC_ASSERT(index >= 1 && index <= 6))
    {
        return;
    }

    if (!AGC_ASSERT(v <= EW15_MAX))
    {
        return;
    }

    g_aotel[index - 1] = v;
}

agc_sp_t agc_erase_cduchkwd(void)
{
    return (g_cduchkwd);
}

void agc_erase_set_cduchkwd(agc_sp_t v)
{
    g_cduchkwd = v;
}

agc_dp_t agc_erase_rted1(void)
{
    return (g_rted1);
}

void agc_erase_set_rted1(agc_dp_t v)
{
    g_rted1 = v;
}

agc_sp_t agc_erase_return_to_earth_range(void)
{
    return (g_p37range);
}

void agc_erase_set_return_to_earth_range(agc_sp_t v)
{
    g_p37range = v;
}

agc_dp_t agc_erase_tland(void)
{
    return (g_tland);
}

void agc_erase_set_tland(agc_dp_t v)
{
    g_tland = v;
}
