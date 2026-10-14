/**********************************************************************
 * FILE NAME: counters.c
 *
 * PURPOSE:
 *   The hardware counter cells and the radar / hand-controller input
 *   registers that several lanes read or drive: P-/Q-/R-RHCCTR, RNRAD,
 *   INLINK, CDUTCMD = TVCYAW, CDUSCMD = TVCPITCH, THRUST, ALTM and
 *   TEMX..TEMZ. The AGC hardware counts these cells in unprogrammed
 *   sequences; here the sim harness or the owning program stores them
 *   through the setters.
 *
 * AGC SOURCE:
 *   Comanche055 ERASABLE_ASSIGNMENTS.agc pp. 39-40, 79 (RNRAD, INLINK,
 *   CDUTCMD, TVCYAW, CDUSCMD, TVCPITCH, THRUST, TEMX)
 *   Luminary099 ERASABLE_ASSIGNMENTS.agc pp. 92-93, 105 (P-RHCCTR, RNRAD,
 *   INLINK, CDUTCMD, THRUST, ALTM, TEMX)
 *   Comanche055 SERVICER207.agc p. 832 and Luminary099 SERVICER.agc
 *   p. 869 (PIPASR +3); INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc (the
 *   counter channels)
 *
 * EXTERNAL VARIABLES:  (module-static)
 *   Name         Type      I/O  Scale             Description
 *   -----------  --------  ---  ----------------  ---------------------
 *   g_rhcctr     agc_sp_t  I/O  counts, unscaled  P-, Q-, R-RHCCTR (042-044, LM)
 *   g_rnrad      uint16_t  I/O  raw 15-bit word   RNRAD (046): radar reading
 *   g_inlink     uint16_t  I/O  raw 15-bit word   INLINK (045): uplink word
 *   g_cducmd     agc_sp_t  I/O  pulses, unscaled  CDUTCMD / CDUSCMD (053, 054)
 *   g_thrust     agc_sp_t  I/O  pulses, unscaled  THRUST (055, LM)
 *   g_altm       uint16_t  I/O  raw 15-bit word   ALTM (060, LM)
 *   g_tem        agc_sp_t  I/O  pulses, unscaled  TEMX, TEMY, TEMZ
 *
 * EXTERNAL REFERENCES:
 *   Name                 Description
 *   -------------------  -------------------------------------------
 *   agc_erase_pipax ...  PIPA registers (erasable.c)
 *   agc_erase_set_delv   DELV, the PIPA counts for the program (erasable.c)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   No AGC alarms and no status returns. The accessors take the AGC's
 *   own indices (an axis 0..2, a CDU command cell 0..1) from constants or
 *   bounded loops, so they hold no checks.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   TVCYAW and CDUTCMD are ONE cell, as are TVCPITCH and CDUSCMD (EQUALS
 *   in the CSM erasable assignments); both spellings use the same storage
 *   here. CSM TVC mode counts 85.41 arc-sec per bit; in the LM the same
 *   cells are the RR trunnion and shaft error counters.
 *
 * NOTES:
 *   042-044 P-RHCCTR, Q-RHCCTR, R-RHCCTR (LM): rotational hand controller
 *   counts, SP signed. 046 RNRAD: the radar reading, raw 15-bit word.
 *   055 THRUST (LM descent engine throttle pulses), SP signed. 060 ALTM
 *   (LM altitude meter / tape meter drive), raw 15-bit word.
 *
 * ALGORITHM:
 *   none (storage), except agc_pipasr_noscale: read and zero the PIPA
 *   counters, keep the counts in TEMX..TEMZ and put them in the high
 *   words of DELV.
 **********************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/interp.h"
#include "agc/core/state.h"

#define CTR_RHC_N    3 /* P, Q, R                                 */
#define CTR_CDUCMD_N 2 /* 0 CDUTCMD (TVCYAW), 1 CDUSCMD (TVCPITCH) */

static agc_sp_t g_rhcctr[CTR_RHC_N];    /* P-, Q-, R-RHCCTR, counts, unscaled (int) */
static uint16_t g_rnrad;                /* RNRAD, raw 15-bit word         */
static uint16_t g_inlink;               /* INLINK, raw 15-bit word        */
static agc_sp_t g_cducmd[CTR_CDUCMD_N]; /* CDUTCMD, CDUSCMD, pulses, unscaled (int) */
static agc_sp_t g_thrust;               /* THRUST, pulses, unscaled (int) */
static uint16_t g_altm;                 /* ALTM, raw 15-bit word          */
static agc_sp_t g_tem[3];               /* TEMX, TEMY, TEMZ, PIPA pulses, unscaled (int) */

/**********************************************************************
 * AGC: none (fresh start) -- zero every counter cell (called by
 *   agc_erase_reset)
 *
 * FUNCTION NAME: agc_erase_reset_counters
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_reset_counters(void)
{
    for (int i = 0; i < CTR_RHC_N; i++)
    {
        g_rhcctr[i] = 0;
    }

    for (int i = 0; i < CTR_CDUCMD_N; i++)
    {
        g_cducmd[i] = 0;
    }

    for (int i = 0; i < 3; i++)
    {
        g_tem[i] = 0;
    }

    g_rnrad  = 0;
    g_inlink = 0;
    g_thrust = 0;
    g_altm   = 0;
}

/**********************************************************************
 * AGC: none (helper of the RHCCTR accessors) -- read one rotational
 *   hand controller count
 *
 * FUNCTION NAME: rhc_get
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   axis      int   I    0 = P, 1 = Q, 2 = R
 *
 * RETURNS: agc_sp_t -- RHC count of the axis, counts, unscaled (int)
 **********************************************************************/
static agc_sp_t rhc_get(int axis)
{
    return (g_rhcctr[axis]);
}

/**********************************************************************
 * AGC: none (helper of the RHCCTR accessors) -- store one rotational
 *   hand controller count
 *
 * FUNCTION NAME: rhc_set
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   axis      int       I    0 = P, 1 = Q, 2 = R
 *   v         agc_sp_t  I    RHC count, counts, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
static void rhc_set(int axis, agc_sp_t v)
{
    g_rhcctr[axis] = v;
}

/**********************************************************************
 * AGC: P-RHCCTR/Q-RHCCTR/R-RHCCTR (ERASABLE_ASSIGNMENTS.agc L p. 92)
 *   -- read P-RHCCTR, the P-axis hand controller count (LM)
 *
 * FUNCTION NAME: agc_erase_p_rhcctr
 *
 * RETURNS: agc_sp_t -- P-RHCCTR, counts, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_p_rhcctr(void)
{
    return (rhc_get(0));
}

/**********************************************************************
 * AGC: P-RHCCTR/Q-RHCCTR/R-RHCCTR (ERASABLE_ASSIGNMENTS.agc L p. 92)
 *   -- read Q-RHCCTR, the Q-axis hand controller count (LM)
 *
 * FUNCTION NAME: agc_erase_q_rhcctr
 *
 * RETURNS: agc_sp_t -- Q-RHCCTR, counts, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_q_rhcctr(void)
{
    return (rhc_get(1));
}

/**********************************************************************
 * AGC: P-RHCCTR/Q-RHCCTR/R-RHCCTR (ERASABLE_ASSIGNMENTS.agc L p. 92)
 *   -- read R-RHCCTR, the R-axis hand controller count (LM)
 *
 * FUNCTION NAME: agc_erase_r_rhcctr
 *
 * RETURNS: agc_sp_t -- R-RHCCTR, counts, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_r_rhcctr(void)
{
    return (rhc_get(2));
}

/**********************************************************************
 * AGC: P-RHCCTR/Q-RHCCTR/R-RHCCTR (ERASABLE_ASSIGNMENTS.agc L p. 92)
 *   -- store P-RHCCTR (the sim or the program counts the hand
 *   controller)
 *
 * FUNCTION NAME: agc_erase_set_p_rhcctr
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    P-RHCCTR, counts, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_p_rhcctr(agc_sp_t v)
{
    rhc_set(0, v);
}

/**********************************************************************
 * AGC: P-RHCCTR/Q-RHCCTR/R-RHCCTR (ERASABLE_ASSIGNMENTS.agc L p. 92)
 *   -- store Q-RHCCTR (the sim or the program counts the hand
 *   controller)
 *
 * FUNCTION NAME: agc_erase_set_q_rhcctr
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    Q-RHCCTR, counts, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_q_rhcctr(agc_sp_t v)
{
    rhc_set(1, v);
}

/**********************************************************************
 * AGC: P-RHCCTR/Q-RHCCTR/R-RHCCTR (ERASABLE_ASSIGNMENTS.agc L p. 92)
 *   -- store R-RHCCTR (the sim or the program counts the hand
 *   controller)
 *
 * FUNCTION NAME: agc_erase_set_r_rhcctr
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    R-RHCCTR, counts, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_r_rhcctr(agc_sp_t v)
{
    rhc_set(2, v);
}

/**********************************************************************
 * AGC: RNRAD (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read
 *   RNRAD, the radar reading (rendezvous / landing radar, VHF ranging)
 *
 * FUNCTION NAME: agc_erase_rnrad
 *
 * RETURNS: uint16_t -- RNRAD, raw 15-bit counter word (scale set by
 *          the radar mode)
 **********************************************************************/
uint16_t agc_erase_rnrad(void)
{
    return (g_rnrad);
}

/**********************************************************************
 * AGC: RNRAD (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store
 *   RNRAD
 *
 * FUNCTION NAME: agc_erase_set_rnrad
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         uint16_t  I    radar reading, raw 15-bit counter word
 *                            (scale set by the radar mode)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_rnrad(uint16_t v)
{
    g_rnrad = v;
}

/**********************************************************************
 * AGC: CDUTCMD/CDUSCMD (ERASABLE_ASSIGNMENTS.agc C p. 40, L p. 92) --
 *   read a CDU command cell (the CSM TVC and LM RR error counters
 *   share them)
 *
 * FUNCTION NAME: agc_erase_cdutcmd
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   which     int   I    0 = CDUTCMD (TVCYAW), 1 = CDUSCMD (TVCPITCH)
 *
 * RETURNS: agc_sp_t -- the cell, pulse count, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_cdutcmd(int which)
{
    return (g_cducmd[which]);
}

/**********************************************************************
 * AGC: CDUTCMD/CDUSCMD (ERASABLE_ASSIGNMENTS.agc C p. 40, L p. 92) --
 *   store a CDU command cell
 *
 * FUNCTION NAME: agc_erase_set_cdutcmd
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   which     int       I    0 = CDUTCMD (TVCYAW), 1 = CDUSCMD
 *                            (TVCPITCH)
 *   v         agc_sp_t  I    pulse count, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_cdutcmd(int which, agc_sp_t v)
{
    g_cducmd[which] = v;
}

/**********************************************************************
 * AGC: TVCYAW (ERASABLE_ASSIGNMENTS.agc C p. 40; = CDUTCMD) -- read
 *   TVCYAW, the SPS yaw command in TVC mode
 *
 * FUNCTION NAME: agc_erase_tvcyaw
 *
 * RETURNS: agc_sp_t -- TVCYAW, pulse count, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_tvcyaw(void)
{
    return (agc_erase_cdutcmd(0));
}

/**********************************************************************
 * AGC: TVCYAW (ERASABLE_ASSIGNMENTS.agc C p. 40; = CDUTCMD) -- store
 *   TVCYAW
 *
 * FUNCTION NAME: agc_erase_set_tvcyaw
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    pulse count, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_tvcyaw(agc_sp_t v)
{
    agc_erase_set_cdutcmd(0, v);
}

/**********************************************************************
 * AGC: TVCPITCH (ERASABLE_ASSIGNMENTS.agc C p. 40; = CDUSCMD) -- read
 *   TVCPITCH, the SPS pitch command in TVC mode
 *
 * FUNCTION NAME: agc_erase_tvcpitch
 *
 * RETURNS: agc_sp_t -- TVCPITCH, pulse count, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_tvcpitch(void)
{
    return (agc_erase_cdutcmd(1));
}

/**********************************************************************
 * AGC: TVCPITCH (ERASABLE_ASSIGNMENTS.agc C p. 40; = CDUSCMD) -- store
 *   TVCPITCH
 *
 * FUNCTION NAME: agc_erase_set_tvcpitch
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         agc_sp_t  I    pulse count, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_tvcpitch(agc_sp_t v)
{
    agc_erase_set_cdutcmd(1, v);
}

/**********************************************************************
 * AGC: THRUST (ERASABLE_ASSIGNMENTS.agc C p. 40, L p. 93) -- read
 *   THRUST, the descent engine throttle pulse cell (LM)
 *
 * FUNCTION NAME: agc_erase_thrust
 *
 * RETURNS: agc_sp_t -- THRUST, pulses, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_thrust(void)
{
    return (g_thrust);
}

/**********************************************************************
 * AGC: THRUST (ERASABLE_ASSIGNMENTS.agc C p. 40, L p. 93) -- store
 *   THRUST
 *
 * FUNCTION NAME: agc_erase_set_thrust
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   pulses    agc_sp_t  I    throttle pulses, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_thrust(agc_sp_t pulses)
{
    g_thrust = pulses;
}

/**********************************************************************
 * AGC: ALTM (ERASABLE_ASSIGNMENTS.agc L p. 93) -- read ALTM, the
 *   altitude / tape meter drive (LM)
 *
 * FUNCTION NAME: agc_erase_altm
 *
 * RETURNS: uint16_t -- ALTM, raw 15-bit word
 **********************************************************************/
uint16_t agc_erase_altm(void)
{
    return (g_altm);
}

/**********************************************************************
 * AGC: ALTM (ERASABLE_ASSIGNMENTS.agc L p. 93) -- store ALTM
 *
 * FUNCTION NAME: agc_erase_set_altm
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   word      uint16_t  I    meter drive, raw 15-bit word
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_altm(uint16_t word)
{
    g_altm = word;
}

/**********************************************************************
 * AGC: PIPASR +3 (SERVICER207.agc C p. 832, L p. 869) -- read and zero
 *   the PIPA counters, keep the counts in TEMX..TEMZ and in the high
 *   words of DELV
 *
 * FUNCTION NAME: agc_pipasr_noscale
 *
 * RETURNS: none
 **********************************************************************/
void agc_pipasr_noscale(void)
{
    agc_sp_t p[3] = {agc_erase_pipax(), agc_erase_pipay(), agc_erase_pipaz()};

    agc_erase_set_pipax(0); /* PIPAS SET TO ZERO AS READ */
    agc_erase_set_pipay(0);
    agc_erase_set_pipaz(0);

    for (int i = 0; i < 3; i++)
    {
        g_tem[i] = p[i];
    }

    agc_erase_set_delv(agc_vec_make(agc_dp_pack(p[0], 0), agc_dp_pack(p[1], 0), agc_dp_pack(p[2], 0)));
}

/**********************************************************************
 * AGC: TEMX (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- read
 *   TEMX/TEMY/TEMZ, the PIPA counts kept by PIPASR
 *
 * FUNCTION NAME: agc_erase_tem
 *
 * ARGUMENTS:
 *   Argument  Type  I/O  Description (scale)
 *   --------  ----  ---  ------------------------------------
 *   axis      int   I    0 = X, 1 = Y, 2 = Z
 *
 * RETURNS: agc_sp_t -- the axis count, PIPA pulses, unscaled (int)
 **********************************************************************/
agc_sp_t agc_erase_tem(int axis)
{
    return (g_tem[axis]);
}

/**********************************************************************
 * AGC: TEMX (ERASABLE_ASSIGNMENTS.agc C p. 79, L p. 105) -- store
 *   TEMX/TEMY/TEMZ
 *
 * FUNCTION NAME: agc_erase_set_tem
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   axis      int       I    0 = X, 1 = Y, 2 = Z
 *   v         agc_sp_t  I    PIPA count, pulses, unscaled (int)
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_tem(int axis, agc_sp_t v)
{
    g_tem[axis] = v;
}

/**********************************************************************
 * AGC: INLINK (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- read
 *   INLINK, the uplink (or nav DSKY keying) word
 *
 * FUNCTION NAME: agc_erase_inlink
 *
 * RETURNS: uint16_t -- INLINK, raw 15-bit word
 **********************************************************************/
uint16_t agc_erase_inlink(void)
{
    return (g_inlink);
}

/**********************************************************************
 * AGC: INLINK (ERASABLE_ASSIGNMENTS.agc C p. 39, L p. 92) -- store
 *   INLINK (the sim stores it and raises UPRUPT)
 *
 * FUNCTION NAME: agc_erase_set_inlink
 *
 * ARGUMENTS:
 *   Argument  Type      I/O  Description (scale)
 *   --------  --------  ---  ------------------------------------
 *   v         uint16_t  I    uplink word, raw 15-bit word
 *
 * RETURNS: none
 **********************************************************************/
void agc_erase_set_inlink(uint16_t v)
{
    g_inlink = v;
}
