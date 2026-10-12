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

void agc_erase_reset_restart_delays(void)
{
    g_p40tmp   = 0;
    g_savet_30 = 0;
    g_zoomtime = 0;
    g_s61dt    = 0;
    g_cm_gymdt = 0;
}

agc_dp_t agc_erase_p40tmp(void)
{
    return (g_p40tmp);
}

void agc_erase_set_p40tmp(agc_dp_t v)
{
    g_p40tmp = v;
}

agc_dp_t agc_erase_savet_30(void)
{
    return (g_savet_30);
}

void agc_erase_set_savet_30(agc_dp_t v)
{
    g_savet_30 = v;
}

agc_sp_t agc_erase_zoomtime(void)
{
    return (g_zoomtime);
}

void agc_erase_set_zoomtime(agc_sp_t v)
{
    g_zoomtime = v;
}

agc_sp_t agc_erase_s61dt(void)
{
    return (g_s61dt);
}

void agc_erase_set_s61dt(agc_sp_t v)
{
    g_s61dt = v;
}

agc_sp_t agc_erase_cm_gymdt(void)
{
    return (g_cm_gymdt);
}

void agc_erase_set_cm_gymdt(agc_sp_t v)
{
    g_cm_gymdt = v;
}
