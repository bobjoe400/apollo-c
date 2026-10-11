/*
 * restart_delays.c -- the erasables the restart tables read as delta
 * times (RESTART_TABLES.agc: GENADR entries), written by the programs
 * that set the phases: P40TMP (CSM P40, 4.21 / 4.57), SAVET-30 (LM P40/
 * P42, 1.3 / 4.25), ZOOMTIME (LM pad load, 3.3), S61DT (CSM P61-P67,
 * 4.43) and CM/GYMDT (CSM entry DAP, 6.11). Kept in core so that the
 * restart code (exec) reads them without calling up.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"

static agc_dp_t g_p40tmp;   /* P40TMP: LONGCALL / TIGON delay, cs 2^28 */
static agc_dp_t g_savet_30; /* SAVET-30: delay to TIG-35, cs 2^28     */
static agc_sp_t g_zoomtime; /* ZOOMTIME: DPS throttle-up time, cs (PL)*/
static agc_sp_t g_s61dt;    /* S61DT: 4.43 waitlist dt, cs            */
static agc_sp_t g_cm_gymdt; /* CM/GYMDT: 6.11 waitlist dt, cs         */

static bool dp_ok(agc_dp_t v)
{
    return ((v >= AGC_DP_MIN) && (v <= AGC_DP_MAX));
}

static bool dt_ok(agc_sp_t v)
{
    return ((v >= 0) && (v <= AGC_SP_MAX));
}

void agc_erase_reset_restart_delays(void)
{
    g_p40tmp   = 0;
    g_savet_30 = 0;
    g_zoomtime = 0;
    g_s61dt    = 0;
    g_cm_gymdt = 0;

    if (!AGC_ASSERT(g_p40tmp == 0 && g_cm_gymdt == 0))
    {
        return;
    }
}

agc_dp_t agc_erase_p40tmp(void)
{
    if (!AGC_ASSERT(dp_ok(g_p40tmp)))
    {
        return (0);
    }

    return (g_p40tmp);
}

void agc_erase_set_p40tmp(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v)))
    {
        return;
    }

    g_p40tmp = v;
}

agc_dp_t agc_erase_savet_30(void)
{
    if (!AGC_ASSERT(dp_ok(g_savet_30)))
    {
        return (0);
    }

    return (g_savet_30);
}

void agc_erase_set_savet_30(agc_dp_t v)
{
    if (!AGC_ASSERT(dp_ok(v)))
    {
        return;
    }

    g_savet_30 = v;
}

agc_sp_t agc_erase_zoomtime(void)
{
    if (!AGC_ASSERT(dt_ok(g_zoomtime)))
    {
        return (0);
    }

    return (g_zoomtime);
}

void agc_erase_set_zoomtime(agc_sp_t v)
{
    if (!AGC_ASSERT(dt_ok(v)))
    {
        return;
    }

    g_zoomtime = v;
}

agc_sp_t agc_erase_s61dt(void)
{
    if (!AGC_ASSERT(dt_ok(g_s61dt)))
    {
        return (0);
    }

    return (g_s61dt);
}

void agc_erase_set_s61dt(agc_sp_t v)
{
    if (!AGC_ASSERT(dt_ok(v)))
    {
        return;
    }

    g_s61dt = v;
}

agc_sp_t agc_erase_cm_gymdt(void)
{
    if (!AGC_ASSERT(dt_ok(g_cm_gymdt)))
    {
        return (0);
    }

    return (g_cm_gymdt);
}

void agc_erase_set_cm_gymdt(agc_sp_t v)
{
    if (!AGC_ASSERT(dt_ok(v)))
    {
        return;
    }

    g_cm_gymdt = v;
}
