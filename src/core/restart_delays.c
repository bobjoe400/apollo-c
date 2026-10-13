/**********************************************************************
 * FILE NAME: restart_delays.c
 *
 * PURPOSE:
 *   The erasables the restart tables read as delta times
 *   (RESTART_TABLES.agc: GENADR entries), written by the programs that
 *   set the phases: P40TMP (CSM P40, 4.21 / 4.57), SAVET-30 (LM P40/P42,
 *   1.3 / 4.25), ZOOMTIME (LM pad load, 3.3), S61DT (CSM P61-P67, 4.43)
 *   and CM/GYMDT (CSM entry DAP, 6.11). Kept in core so that the restart
 *   code (exec) reads them without calling up.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 112, 116, 124; RESTART_TABLES.agc
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 138, 147; RESTART_TABLES.agc
 *
 * EXTERNAL VARIABLES:  (module-static)
 *   Name         Type      I/O  Scale             Description
 *   -----------  --------  ---  ----------------  ---------------------
 *   g_p40tmp     agc_dp_t  I/O  cs, 2^28          P40TMP
 *   g_savet_30   agc_dp_t  I/O  cs, 2^28          SAVET-30
 *   g_zoomtime   agc_sp_t  I/O  cs, unscaled      ZOOMTIME
 *   g_s61dt      agc_sp_t  I/O  cs, unscaled      S61DT
 *   g_cm_gymdt   agc_sp_t  I/O  cs, unscaled      CM/GYMDT
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No AGC alarms and no status returns. The accessors hold no checks:
 *   the delays are the values the setting programs computed.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The vehicle-specific cells are present for both vehicles and used
 *   by one.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assertion.h"
#include "agc/core/state.h"

static agc_dp_t g_p40tmp;   /* P40TMP: LONGCALL / TIGON delay, cs 2^28 */
static agc_dp_t g_savet_30; /* SAVET-30: delay to TIG-35, cs 2^28     */
static agc_sp_t g_zoomtime; /* ZOOMTIME: DPS throttle-up time, cs, unscaled (PL) */
static agc_sp_t g_s61dt;    /* S61DT: 4.43 waitlist dt, cs, unscaled */
static agc_sp_t g_cm_gymdt; /* CM/GYMDT: 6.11 waitlist dt, cs, unscaled */

/**********************************************************************
 * AGC: none (fresh start) -- zero the restart delay cells (called by
 *   agc_erase_reset)
 *
 * FUNCTION NAME: agc_erase_reset_restart_delays
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_reset_restart_delays(void)
{
    g_p40tmp   = 0;
    g_savet_30 = 0;
    g_zoomtime = 0;
    g_s61dt    = 0;
    g_cm_gymdt = 0;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 124 -- read P40TMP, the LONGCALL
 *   / TIGON delay of CSM P40 (RESTART_TABLES.agc 4.21 / 4.57)
 *
 * FUNCTION NAME: agc_erase_p40tmp
 *
 * RETURNS: agc_dp_t -- P40TMP, cs, scale 2^28
 **********************************************************************/
agc_dp_t agc_erase_p40tmp(void)
{
    return (g_p40tmp);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 124 -- store P40TMP, the LONGCALL
 *   / TIGON delay of CSM P40 (RESTART_TABLES.agc 4.21 / 4.57)
 *
 * FUNCTION NAME: agc_erase_set_p40tmp
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    P40TMP, cs, scale 2^28
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_p40tmp(agc_dp_t v)
{
    g_p40tmp = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 147 -- read SAVET-30, the delay
 *   to TIG-35 of LM P40 / P42 (restart groups 1.3 / 4.25)
 *
 * FUNCTION NAME: agc_erase_savet_30
 *
 * RETURNS: agc_dp_t -- SAVET-30, cs, scale 2^28
 **********************************************************************/
agc_dp_t agc_erase_savet_30(void)
{
    return (g_savet_30);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 147 -- store SAVET-30, the delay
 *   to TIG-35 of LM P40 / P42 (restart groups 1.3 / 4.25)
 *
 * FUNCTION NAME: agc_erase_set_savet_30
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    SAVET-30, cs, scale 2^28
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_savet_30(agc_dp_t v)
{
    g_savet_30 = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 138 -- read ZOOMTIME, the DPS
 *   throttle-up time (LM pad load, restart group 3.3)
 *
 * FUNCTION NAME: agc_erase_zoomtime
 *
 * RETURNS: agc_sp_t -- ZOOMTIME, cs, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_zoomtime(void)
{
    return (g_zoomtime);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 138 -- store ZOOMTIME, the DPS
 *   throttle-up time (LM pad load, restart group 3.3)
 *
 * FUNCTION NAME: agc_erase_set_zoomtime
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    ZOOMTIME, cs, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_zoomtime(agc_sp_t v)
{
    g_zoomtime = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 116 -- read S61DT, the variable
 *   waitlist dt of S61.1 (CSM P61-P67, restart group 4.43)
 *
 * FUNCTION NAME: agc_erase_s61dt
 *
 * RETURNS: agc_sp_t -- S61DT, cs, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_s61dt(void)
{
    return (g_s61dt);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 116 -- store S61DT, the variable
 *   waitlist dt of S61.1 (CSM P61-P67, restart group 4.43)
 *
 * FUNCTION NAME: agc_erase_set_s61dt
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    S61DT, cs, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_s61dt(agc_sp_t v)
{
    g_s61dt = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 112 -- read CM/GYMDT, the
 *   waitlist dt of the CSM entry DAP (restart group 6.11)
 *
 * FUNCTION NAME: agc_erase_cm_gymdt
 *
 * RETURNS: agc_sp_t -- CM/GYMDT, cs, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_cm_gymdt(void)
{
    return (g_cm_gymdt);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 112 -- store CM/GYMDT, the
 *   waitlist dt of the CSM entry DAP (restart group 6.11)
 *
 * FUNCTION NAME: agc_erase_set_cm_gymdt
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    CM/GYMDT, cs, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_cm_gymdt(agc_sp_t v)
{
    g_cm_gymdt = v;
}
