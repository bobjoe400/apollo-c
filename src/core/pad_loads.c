/**********************************************************************
 * FILE NAME: pad_loads.c
 *
 * PURPOSE:
 *   Pad-loaded erasables (B(n)PL / I(n)PL) read by several lanes: AOTAZ
 *   and AOTEL (LM), CDUCHKWD, RTED1 and P37RANGE (CSM) and TLAND (LM).
 *   Accessors in the style of erasable.c: one static object per AGC
 *   erasable, scale factors in include/agc/core/state.h.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 82, 83, 116
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 121, 138
 *
 * EXTERNAL VARIABLES:  (module-static)
 *   Name         Type      I/O  Scale              Description
 *   -----------  --------  ---  -----------------  ---------------------
 *   g_aotaz      uint16_t  I/O  CDU word, 2^15 rev AOT detent azimuths 1..6
 *   g_aotel      uint16_t  I/O  CDU word, 2^15 rev AOT detent elevations 1..6
 *   g_cduchkwd   agc_sp_t  I/O  cs, unscaled      CDU check delay
 *   g_rted1      agc_dp_t  I/O  2^-3 (B-3)        VGAMMA coefficient D1
 *   g_p37range   agc_sp_t  I/O  revs, 2^0         P37 range override
 *   g_tland      agc_dp_t  I/O  cs, 2^28          nominal time of landing
 *
 * EXTERNAL REFERENCES:
 *   none
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No AGC alarms and no status returns. The accessors hold no checks:
 *   the detent index is the AGC's own 1..6 from constants or bounded
 *   loops and a pad load holds whatever word the ground stored.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   The vehicle-specific cells are present for both vehicles and used
 *   by one.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/state.h"
#include "internal.h"

#define EW15_MAX 077777u /* a raw 15-bit AGC word */

static uint16_t g_aotaz[6]; /* AOTAZ -1+n (LM, B(6)PL): AOT detent n = 1..6 azimuth, CDU words (2^15 = 1 revolution) */
static uint16_t g_aotel[6]; /* AOTEL -1+n (LM, B(6)PL): AOT detent n = 1..6 elevation, CDU words (2^15 = 1 revolution) */
static agc_sp_t g_cduchkwd; /* CDUCHKWD (CSM, B(1)PL): CDU check delay for R60 / IMU checks, cs, unscaled */
static agc_dp_t g_rted1;    /* RTED1 (CSM, I(2)PL): VGAMMA polynomial coefficient D1 of P37, 2^-3 (B-3) */
static agc_sp_t g_p37range; /* P37RANGE (= R61CNTR +1, CSM I(1)PL): entry range override, revs 2^0 */
static agc_dp_t g_tland;    /* TLAND (= W, LM I(2)): nominal time of landing, cs 2^28 */

/**********************************************************************
 * AGC: none (fresh start) -- zero the pad-loaded erasables (called by
 *   agc_erase_reset)
 *
 * FUNCTION NAME: agc_erase_reset_pad_loads
 *
 * RETURNS: none
 **********************************************************************/
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

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 138 -- read AOTAZ -1+n, the AOT
 *   detent azimuth (LM, B(6)PL)
 *
 * FUNCTION NAME: agc_erase_aotaz
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   index     int   I    AOT detent n, 1..6
 *
 * RETURNS: uint16_t -- AOTAZ, CDU word (2^15 = 1 revolution)
 **********************************************************************/
uint16_t agc_erase_aotaz(int index)
{
    return (g_aotaz[index - 1]);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 138 -- store AOTAZ -1+n, the AOT
 *   detent azimuth (LM, B(6)PL)
 *
 * FUNCTION NAME: agc_erase_set_aotaz
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   index     int       I    AOT detent n, 1..6
 *   v         uint16_t  I    AOTAZ, CDU word (2^15 = 1 revolution)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_aotaz(int index, uint16_t v)
{
    g_aotaz[index - 1] = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 138 -- read AOTEL -1+n, the AOT
 *   detent elevation (LM, B(6)PL)
 *
 * FUNCTION NAME: agc_erase_aotel
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   index     int   I    AOT detent n, 1..6
 *
 * RETURNS: uint16_t -- AOTEL, CDU word (2^15 = 1 revolution)
 **********************************************************************/
uint16_t agc_erase_aotel(int index)
{
    return (g_aotel[index - 1]);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 138 -- store AOTEL -1+n, the AOT
 *   detent elevation (LM, B(6)PL)
 *
 * FUNCTION NAME: agc_erase_set_aotel
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   index     int       I    AOT detent n, 1..6
 *   v         uint16_t  I    AOTEL, CDU word (2^15 = 1 revolution)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_aotel(int index, uint16_t v)
{
    g_aotel[index - 1] = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 82 -- read CDUCHKWD, the CDU
 *   check delay for the R60 / IMU checks (CSM, B(1)PL)
 *
 * FUNCTION NAME: agc_erase_cduchkwd
 *
 * RETURNS: agc_sp_t -- CDUCHKWD, cs, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_cduchkwd(void)
{
    return (g_cduchkwd);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 82 -- store CDUCHKWD, the CDU
 *   check delay for the R60 / IMU checks (CSM, B(1)PL)
 *
 * FUNCTION NAME: agc_erase_set_cduchkwd
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    CDUCHKWD, cs, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_cduchkwd(agc_sp_t v)
{
    g_cduchkwd = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 83 -- read RTED1, the VGAMMA
 *   polynomial coefficient D1 of P37 (CSM, I(2)PL)
 *
 * FUNCTION NAME: agc_erase_rted1
 *
 * RETURNS: agc_dp_t -- RTED1, scale 2^-3 (B-3)
 **********************************************************************/
agc_dp_t agc_erase_rted1(void)
{
    return (g_rted1);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 83 -- store RTED1, the VGAMMA
 *   polynomial coefficient D1 of P37 (CSM, I(2)PL)
 *
 * FUNCTION NAME: agc_erase_set_rted1
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    RTED1, scale 2^-3 (B-3)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_rted1(agc_dp_t v)
{
    g_rted1 = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 116 -- read P37RANGE (= R61CNTR
 *   +1), the P37 entry range override (CSM, I(1)PL)
 *
 * FUNCTION NAME: agc_erase_return_to_earth_range
 *
 * RETURNS: agc_sp_t -- P37RANGE, revolutions, scale 2^0
 **********************************************************************/
agc_sp_t agc_erase_return_to_earth_range(void)
{
    return (g_p37range);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc C p. 116 -- store P37RANGE (= R61CNTR
 *   +1), the P37 entry range override (CSM, I(1)PL)
 *
 * FUNCTION NAME: agc_erase_set_return_to_earth_range
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    P37RANGE, revolutions, scale 2^0
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_return_to_earth_range(agc_sp_t v)
{
    g_p37range = v;
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 121 -- read TLAND (= W), the
 *   nominal time of landing (LM, I(2))
 *
 * FUNCTION NAME: agc_erase_tland
 *
 * RETURNS: agc_dp_t -- TLAND, cs, scale 2^28
 **********************************************************************/
agc_dp_t agc_erase_tland(void)
{
    return (g_tland);
}

/**********************************************************************
 * AGC: ERASABLE_ASSIGNMENTS.agc L p. 121 -- store TLAND (= W), the
 *   nominal time of landing (LM, I(2))
 *
 * FUNCTION NAME: agc_erase_set_tland
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_dp_t  I    TLAND, cs, scale 2^28
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_tland(agc_dp_t v)
{
    g_tland = v;
}
