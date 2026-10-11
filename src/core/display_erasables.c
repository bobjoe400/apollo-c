/*
 * display_erasables.c -- display and downlink registers shared by pinball and the programs.
 *
 * Accessors in the style of erasable.c: one static object per AGC erasable,
 * a range check on every store, scale factors in include/agc/core/state.h.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"

#define EW15_MAX 077777u /* a raw 15-bit AGC word */

/* OPTION1 +0..+2 = OPTION1, OPTION2, OPTION3 (noun 06), octal words.     */
static agc_sp_t g_option[3];

/* SAMPTIME, SAMPTIME +1: sampled TIME2, TIME1 (noun 65), cs 2^28.         */
static agc_dp_t g_samptime;

/* SFAIL (= SELFERAS): self-check failure address, octal word.             */
static uint16_t g_sfail;

/* MARKDOWN +0..+6 (CSM P22/P23/R21 mark data for the downlink), MARK2DWN
 * +0..+6 (second mark of the alignment P50s), raw words.                  */
static uint16_t g_markdown[7];
static uint16_t g_mark2dwn[7];

/* TRKMKCNT: rendezvous tracking mark counter (noun 45 R1), whole count.   */
static agc_sp_t g_trkmkcnt;

/* T3TOT4 (= RELDELV +2): TPF time for N39 (P34/P35/P37), cs 2^28.         */
static agc_dp_t g_t3tot4;

/* RR-AZ, RR-ELEV (LM, noun 56): RR LOS azimuth / elevation, revs 2^0.     */
static agc_dp_t g_rr_az;
static agc_dp_t g_rr_elev;

void agc_erase_reset_display(void)
{
    for (int i = 0; i < 7; i++)
    {
        g_markdown[i] = 0;
        g_mark2dwn[i] = 0;
    }

    for (int i = 0; i < 3; i++)
    {
        g_option[i] = 0;
    }

    g_samptime = 0;
    g_sfail    = 0;
    g_trkmkcnt = 0;
    g_t3tot4   = 0;
    g_rr_az    = 0;
    g_rr_elev  = 0;
}

agc_sp_t agc_erase_option1(int index)
{
    return (g_option[index - 0]);
}

void agc_erase_set_option1(int index, agc_sp_t v)
{
    g_option[index - 0] = v;
}

agc_dp_t agc_erase_samptime(void)
{
    return (g_samptime);
}

void agc_erase_set_samptime(agc_dp_t v)
{
    g_samptime = v;
}

uint16_t agc_erase_sfail(void)
{
    return (g_sfail);
}

void agc_erase_set_sfail(uint16_t v)
{
    g_sfail = v;
}

uint16_t agc_erase_markdown(int index)
{
    if (!AGC_ASSERT(index >= 0 && index <= 6))
    {
        return (0);
    }

    if (!AGC_ASSERT(g_markdown[index - 0] <= EW15_MAX))
    {
        return (0);
    }

    return (g_markdown[index - 0]);
}

void agc_erase_set_markdown(int index, uint16_t v)
{
    if (!AGC_ASSERT(v <= EW15_MAX))
    {
        return;
    }

    g_markdown[index - 0] = v;
}

uint16_t agc_erase_mark2dwn(int index)
{
    if (!AGC_ASSERT(index >= 0 && index <= 6))
    {
        return (0);
    }

    if (!AGC_ASSERT(g_mark2dwn[index - 0] <= EW15_MAX))
    {
        return (0);
    }

    return (g_mark2dwn[index - 0]);
}

void agc_erase_set_mark2dwn(int index, uint16_t v)
{
    if (!AGC_ASSERT(v <= EW15_MAX))
    {
        return;
    }

    g_mark2dwn[index - 0] = v;
}

agc_sp_t agc_erase_trkmkcnt(void)
{
    return (g_trkmkcnt);
}

void agc_erase_set_trkmkcnt(agc_sp_t v)
{
    g_trkmkcnt = v;
}

agc_dp_t agc_erase_t3tot4(void)
{
    return (g_t3tot4);
}

void agc_erase_set_t3tot4(agc_dp_t v)
{
    g_t3tot4 = v;
}

agc_dp_t agc_erase_rr_az(void)
{
    return (g_rr_az);
}

void agc_erase_set_rr_az(agc_dp_t v)
{
    g_rr_az = v;
}

agc_dp_t agc_erase_rr_elev(void)
{
    return (g_rr_elev);
}

void agc_erase_set_rr_elev(agc_dp_t v)
{
    g_rr_elev = v;
}
