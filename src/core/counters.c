/*
 * counters.c -- the hardware counter cells and the radar / hand-controller
 * input registers that several lanes read or drive.
 *
 * Source: ERASABLE_ASSIGNMENTS.agc (counter assignments 032-060, both
 * ropes) and INPUT_OUTPUT_CHANNEL_BIT_DESCRIPTIONS.agc. The AGC hardware
 * counts these cells in unprogrammed sequences; here the sim harness or the
 * owning program stores them through the setters.
 *
 *   042-044 P-RHCCTR, Q-RHCCTR, R-RHCCTR (LM): rotational hand controller
 *           counts, SP signed.
 *   046     RNRAD: the radar reading (rendezvous radar / landing radar /
 *           VHF ranging), raw 15-bit word.
 *   053     CDUTCMD = TVCYAW (CSM TVC mode, 85.41 arc-sec/bit; LM: RR
 *           trunnion error counter), SP signed pulse count.
 *   054     CDUSCMD = TVCPITCH (CSM TVC mode; LM: RR shaft error counter).
 *   055     THRUST (LM descent engine throttle pulses), SP signed.
 *   060     ALTM (LM altitude meter / tape meter drive), raw 15-bit word.
 *
 * TVCYAW and CDUTCMD are ONE cell, as are TVCPITCH and CDUSCMD (EQUALS in
 * the CSM erasable assignments); both spellings use the same storage here.
 */
#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"

#define CTR_WORD15_MAX 077777u
#define CTR_RHC_N      3 /* P, Q, R                                 */
#define CTR_CDUCMD_N   2 /* 0 CDUTCMD (TVCYAW), 1 CDUSCMD (TVCPITCH) */

static agc_sp_t g_rhcctr[CTR_RHC_N];    /* P-, Q-, R-RHCCTR, counts       */
static uint16_t g_rnrad;                /* RNRAD, raw 15-bit word         */
static uint16_t g_inlink;               /* INLINK, raw 15-bit word        */
static agc_sp_t g_cducmd[CTR_CDUCMD_N]; /* CDUTCMD, CDUSCMD, pulse counts */
static agc_sp_t g_thrust;               /* THRUST, pulses                 */
static uint16_t g_altm;                 /* ALTM, raw 15-bit word          */
static agc_sp_t g_tem[3];               /* TEMX, TEMY, TEMZ, PIPA pulses  */

/* Zero every counter cell (fresh start; called by agc_erase_reset).      */
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

    g_rnrad = 0;
    g_inlink = 0;
    g_thrust = 0;
    g_altm = 0;
}

/* ---- P-, Q-, R-RHCCTR (LM, 042-044) ------------------------------------ */
static agc_sp_t rhc_get(int axis)
{
    return (g_rhcctr[axis]);
}

static void rhc_set(int axis, agc_sp_t v)
{
    g_rhcctr[axis] = v;
}

agc_sp_t agc_erase_p_rhcctr(void)
{
    return (rhc_get(0));
}

agc_sp_t agc_erase_q_rhcctr(void)
{
    return (rhc_get(1));
}

agc_sp_t agc_erase_r_rhcctr(void)
{
    return (rhc_get(2));
}

void agc_erase_set_p_rhcctr(agc_sp_t v)
{
    rhc_set(0, v);
}

void agc_erase_set_q_rhcctr(agc_sp_t v)
{
    rhc_set(1, v);
}

void agc_erase_set_r_rhcctr(agc_sp_t v)
{
    rhc_set(2, v);
}

/* ---- RNRAD (046) -------------------------------------------------------- */
uint16_t agc_erase_rnrad(void)
{
    return (g_rnrad);
}

void agc_erase_set_rnrad(uint16_t v)
{
    g_rnrad = v;
}

/* ---- CDUTCMD = TVCYAW (053), CDUSCMD = TVCPITCH (054) ------------------- */
agc_sp_t agc_erase_cdutcmd(int which)
{
    return (g_cducmd[which]);
}

void agc_erase_set_cdutcmd(int which, agc_sp_t v)
{
    g_cducmd[which] = v;
}

agc_sp_t agc_erase_tvcyaw(void)
{
    return (agc_erase_cdutcmd(0));
}

void agc_erase_set_tvcyaw(agc_sp_t v)
{
    agc_erase_set_cdutcmd(0, v);
}

agc_sp_t agc_erase_tvcpitch(void)
{
    return (agc_erase_cdutcmd(1));
}

void agc_erase_set_tvcpitch(agc_sp_t v)
{
    agc_erase_set_cdutcmd(1, v);
}

/* ---- THRUST (055) and ALTM (060), LM ------------------------------------ */
agc_sp_t agc_erase_thrust(void)
{
    return (g_thrust);
}

void agc_erase_set_thrust(agc_sp_t pulses)
{
    g_thrust = pulses;
}

uint16_t agc_erase_altm(void)
{
    return (g_altm);
}

void agc_erase_set_altm(uint16_t word)
{
    g_altm = word;
}

/* ---- PIPASR +3 (SERVICER, CM p.832 / LM p.869) --------------------------
 * Read the PIPA counters (DXCH PIPAX / DXCH TEMX, XCH PIPAZ / TS TEMZ),
 * zero them, and put the counts in the high words of DELV (low words zero
 * "for TM indication"). The PIPASR entry proper also loads PIPTIME1
 * (guidance, agc_guid_pipasr); +3 is used by NBDONLY on the surface and by
 * BIBIBIAS. TEMX..TEMZ keep the counts (LM P66 ROD reads them).          */
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

agc_sp_t agc_erase_tem(int axis)
{
    return (g_tem[axis]);
}

void agc_erase_set_tem(int axis, agc_sp_t v)
{
    g_tem[axis] = v;
}

/* ---- INLINK (045): the uplink (or nav DSKY keying) word, raw 15 bits;
 * the sim stores it and raises UPRUPT.                                    */
uint16_t agc_erase_inlink(void)
{
    return (g_inlink);
}

void agc_erase_set_inlink(uint16_t v)
{
    g_inlink = v;
}
