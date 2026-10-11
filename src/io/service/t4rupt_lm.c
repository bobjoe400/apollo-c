/**********************************************************************
 * FILE NAME: t4rupt_lm.c
 *
 * PURPOSE:
 *   The LM-only part of T4RUPT: the rendezvous radar inbit and gimbal
 *   limit monitors (RRAUTCHK, RRCDUCHK, RRGIMON / MONREPOS) and the
 *   gimbal-to-pilot axis matrix for the LM DAP (GPMATRIX = DAPT4S).
 *
 * AGC SOURCE:
 *   Luminary099 T4RUPT_PROGRAM.agc pp.182-189  (RRAUTCHK, RRCDUCHK,
 *   RRGIMON, MONREPOS, GPMATRIX); Comanche055 has no counterpart.
 *
 * EXTERNAL VARIABLES:  (AGC erasables reached through accessors)
 *   Name      Type      I/O  Scale        Description
 *   --------  --------  ---  -----------  ----------------------------
 *   RADMODES  uint16_t  I/O  bits         FLGWRD12, radar mode bits
 *   CDUX/Z    uint16_t  I    2's comp pi  outer / middle gimbal CDU
 *   CDUT/S    uint16_t  I    2's comp pi  RR trunnion / shaft CDU
 *   M11..M32  agc_sp_t  O    1            DAP matrix (owned here)
 *
 * EXTERNAL REFERENCES:
 *   Name               Description
 *   -----------------  ----------------------------------------------
 *   agc_rr_limchk      RRLIMCHK, P20-P25.agc (prog-rendz)
 *   agc_rr_settrkf     SETTRKF: tracker fail lamp (prog-rendz)
 *   agc_waitlist_call  WAITLIST: RRTURNON (10 ms), DORREPOS (20 ms)
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   Alarm 00515 (RR CDU fail while RNDVZFLG, P20/P22).  BAD_ARG when the
 *   waitlist rejects a call.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   LM only; runs inside T4RUPT (agc_t4rupt, t4rupt.c).  T4JUMP runs
 *   RRAUTCHK at phases 1 and 5 (falling into GPMATRIX through NORRGMON)
 *   and DAPT4S at phases 3 and 7, so GPMATRIX runs 4 times a second.
 *
 * NOTES:
 *   RADMODES EQUALS FLGWRD12 (FLAGWORD_ASSIGNMENTS p.85): the accessors
 *   below name the flagword, there is no second copy.
 *
 * ALGORITHM:
 *         | sin MG            0        1 |
 *   M   = | cos MG cos OG     sin OG   0 |     (AOG = CDUX, AMG = CDUZ)
 *    GP   | -cos MG sin OG    cos OG   0 |
 *   SPSIN/SPCOS of the raw CDU words (read as ones' complement, scaled
 *   at pi), the products by EXTEND MP (upper word).
 **********************************************************************/
#include <stdlib.h>

#include <stdbool.h>
#include <stdint.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "agc/programs/rendezvous.h"

#define RADMODES_WORD     12      /* RADMODES EQUALS FLGWRD12          */
#define AUTOMBIT          000002u /* BIT2: RR not in auto (inverted)   */
#define RCDUFBIT          000100u /* BIT7: RR CDU OK                   */
#define REPOSBIT          002000u /* BIT11: reposition in progress     */
#define WORD15            077777u /* one AGC word, 15 bits             */
#define OCT05776          005776u /* keeps all but CDES/REMODE/REPOS.. */
#define OCT10001          010001u /* RRCDUZRO and TURNON bits          */
#define OCT32002          032002u /* remode, zeroing, monitor, no auto */
#define OCT20002          020002u /* tracker and error counter enable  */
#define CDU_SIGN          040000u /* sign bit of a CDU counter         */
#define RRTURNON_DELAY_CS 1u      /* CAF ONE: RRTURNON in 10 ms        */
#define DORREPOS_DELAY_CS 2u      /* CAF TWO: DORREPOS in 20 ms        */
#define ALARM_RR_CDU_FAIL ((agc_alarm_code_t) 00515) /* in P20/P22     */
/* sin^2 + cos^2 = 1 at 2^28; SPSIN/SPCOS are good to a few LSB of 2^14 */
#define UNIT_SQ_TOL ((agc_dp_t) 0x00100000)       /* 2^20 = 2^-8 of 1  */

static agc_sp_t m11;          /* M11 = SIN(MG), scaled at 1             */
static agc_sp_t m21;          /* M21 = COS(OG)COS(MG), scaled at 1      */
static agc_sp_t m22;          /* M22 = SIN(OG) (also MR22), scaled at 1 */
static agc_sp_t m31;          /* M31 = -SIN(OG)COS(MG), scaled at 1     */
static agc_sp_t m32;          /* M32 = COS(OG) (also MR23), scaled at 1 */
static agc_sp_t cosmg;        /* COSMG = COS(MG), scaled at 1           */
static bool     matrix_valid; /* GPMATRIX has run since power-on        */

/**********************************************************************
 * AGC: none (invariants of the GPMATRIX outputs, p.188)
 *
 * |M21| and |M31| never exceed |COSMG| (each is COSMG times a sine or a
 * cosine), and SIN^2 + COS^2 is 1 for both gimbal angles.  Before the
 * first GPMATRIX the elements are the power-on zeros.
 *
 * RETURNS: true when the stored matrix satisfies the invariants
 **********************************************************************/
static bool matrix_consistent(void)
{
    agc_dp_t one = (agc_dp_t) 1 << 28;                   /* 1 at 2^28 */
    agc_dp_t mg  = agc_sp_mul(m11, m11) + agc_sp_mul(cosmg, cosmg); /* 2^28 */
    agc_dp_t og  = agc_sp_mul(m22, m22) + agc_sp_mul(m32, m32); /* 2^28 */
    int      cm  = (cosmg < 0) ? -cosmg : cosmg;         /* |COSMG|   */
    int      a21 = (m21 < 0) ? -m21 : m21;               /* |M21|     */
    int      a31 = (m31 < 0) ? -m31 : m31;               /* |M31|     */

    if (!matrix_valid)
    {
        return (true);
    }

    return ((a21 <= cm + 1) && (a31 <= cm + 1) &&
            (mg > one - UNIT_SQ_TOL) && (mg < one + UNIT_SQ_TOL) &&
            (og > one - UNIT_SQ_TOL) && (og < one + UNIT_SQ_TOL));
}

/**********************************************************************
 * RADMODES (ERASABLE_ASSIGNMENTS; EQUALS FLGWRD12, FLAGWORD_ASSIGNMENTS
 * p.85) -- the radar mode bit word, read
 *
 * RETURNS: RADMODES (15 bits), or 0 in a CSM build
 **********************************************************************/
uint16_t agc_erase_radmodes(void)
{
    uint16_t v = 0; /* RADMODES bits */

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (0); /* no valid result */
    }

    v = agc_flagword_get(RADMODES_WORD);

    if (!AGC_ASSERT(v <= WORD15))
    {
        return ((uint16_t) (v & WORD15));
    }

    return (v);
}

/**********************************************************************
 * RADMODES (FLGWRD12, FLAGWORD_ASSIGNMENTS p.85) -- the radar mode bit
 * word, written
 *
 * ARGUMENTS:
 *   v  uint16_t  I   new RADMODES, 15 bits
 **********************************************************************/
void agc_erase_set_radmodes(uint16_t v)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return;
    }

    if (AGC_ASSERT(v <= WORD15))
    {
        agc_flagword_set(RADMODES_WORD, v);
    }
}

/**********************************************************************
 * M11 (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- SIN(MG), read by the DAP
 *
 * RETURNS: M11, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m11(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM) ||
        !AGC_ASSERT(matrix_consistent()))
    {
        return (0); /* no valid result */
    }

    return (m11);
}

/**********************************************************************
 * M21 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- COS(OG)COS(MG)
 *
 * RETURNS: M21, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m21(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM) ||
        !AGC_ASSERT(matrix_consistent()))
    {
        return (0); /* no valid result */
    }

    return (m21);
}

/**********************************************************************
 * M22 (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- SIN(OG), also MR22
 *
 * RETURNS: M22, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m22(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM) ||
        !AGC_ASSERT(matrix_consistent()))
    {
        return (0); /* no valid result */
    }

    return (m22);
}

/**********************************************************************
 * M31 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- -SIN(OG)COS(MG)
 *
 * RETURNS: M31, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m31(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM) ||
        !AGC_ASSERT(matrix_consistent()))
    {
        return (0); /* no valid result */
    }

    return (m31);
}

/**********************************************************************
 * M32 (GPMATRIX, T4RUPT_PROGRAM.agc p.189) -- COS(OG), also MR23
 *
 * RETURNS: M32, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_m32(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM) ||
        !AGC_ASSERT(matrix_consistent()))
    {
        return (0); /* no valid result */
    }

    return (m32);
}

/**********************************************************************
 * COSMG (GPMATRIX, T4RUPT_PROGRAM.agc p.188) -- COS(MG), a factor only
 *
 * RETURNS: COSMG, scaled at 1
 **********************************************************************/
agc_sp_t agc_erase_cosmg(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM) ||
        !AGC_ASSERT(matrix_consistent()))
    {
        return (0); /* no valid result */
    }

    return (cosmg);
}

/**********************************************************************
 * AGC: none (CAE CDUZ / CAE CDUX in GPMATRIX, p.188) -- a CDU counter as
 * SPSIN sees it
 *
 * The counter is a two's-complement angle; CAE loads the raw word and
 * SPSIN/SPCOS treat it as ones' complement, so a negative angle reads one
 * LSB (2^-14 pi) low in magnitude, as in the AGC.
 *
 * ARGUMENTS:
 *   cdu  uint16_t  I   CDU counter word, 15 bits, 2's complement, 2^0 pi
 *
 * RETURNS: the same word as a ones'-complement SP value, scaled at pi
 **********************************************************************/
static agc_sp_t cdu_sp(uint16_t cdu)
{
    int v = 0; /* signed reading, LSB 2^-14 pi */

    if (!AGC_ASSERT(cdu <= WORD15))
    {
        return (0); /* no valid result */
    }

    v = ((cdu & CDU_SIGN) != 0u) ? -(int) (WORD15 - cdu) : (int) cdu; /* 1s */

    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) v);
}

/**********************************************************************
 * AGC: none (EXTEND / MP in GPMATRIX, p.189) -- upper word of an SP x SP
 * product
 *
 * ARGUMENTS:
 *   a  agc_sp_t  I   multiplicand, scaled at 1
 *   b  agc_sp_t  I   multiplier, scaled at 1
 *
 * RETURNS: the high word of a*b (what MP leaves in A), scaled at 1
 **********************************************************************/
static agc_sp_t mp_hi(agc_sp_t a, agc_sp_t b)
{
    agc_dp_t p = agc_sp_mul(a, b); /* A,L product, scaled at 1 */

    agc_sp_t hi = 0; /* high word, scaled at 1 */

    if (!AGC_ASSERT(p >= AGC_DP_MIN && p <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    hi = agc_dp_high(p);

    /* |a b| <= |a| for |b| <= 1 */
    if (!AGC_ASSERT(abs(hi) <= abs(a)))
    {
        return (0); /* no valid result */
    }

    return (hi);
}

/**********************************************************************
 * GPMATRIX = DAPT4S = NORRGMON (T4RUPT_PROGRAM.agc pp.188-189) -- the SP
 * gimbal-to-pilot matrix elements for the LM DAP
 *
 * RETURNS: AGC_STATUS_OK (ENDDAPT4 = RESUME), or BAD_ARG if the result
 *          breaks the matrix invariants
 **********************************************************************/
agc_status_t agc_t4_gimbal_matrix(void)
{
    agc_sp_t mg = cdu_sp(agc_erase_cduz()); /* AMG, scaled at pi */
    agc_sp_t og = cdu_sp(agc_erase_cdux()); /* AOG, scaled at pi */

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    m11          = agc_sp_sin(mg);                 /* SIN(MG)          */
    cosmg        = agc_sp_cos(mg);                 /* COS(MG)          */
    m22          = agc_sp_sin(og);                 /* SIN(OG)          */
    m31          = mp_hi((agc_sp_t) -m22, cosmg);  /* CS M22, MP COSMG */
    m32          = agc_sp_cos(og);                 /* COS(OG)          */
    m21          = mp_hi(m32, cosmg);              /* MP COSMG         */
    matrix_valid = true;

    if (!AGC_ASSERT(matrix_consistent()))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/**********************************************************************
 * AGC: none (CS bits / EXTEND / WAND chan, pp.184 and 187) -- clear bits
 * of an output channel
 *
 * ARGUMENTS:
 *   chan  int       I   channel number (CHAN12 here)
 *   bits  uint16_t  I   bits to clear, 15 bits
 **********************************************************************/
static void ch_clear(int chan, uint16_t bits)
{
    if (!AGC_ASSERT(bits <= WORD15))
    {
        return;
    }

    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & WORD15));

    if (!AGC_ASSERT((agc_chan_read(chan) & bits) == 0u))
    {
        return;
    }
}

/**********************************************************************
 * RRGIMON / MONREPOS (T4RUPT_PROGRAM.agc pp.186-187) -- RR gimbal limit
 * monitor (R25)
 *
 * Skipped when NORRMON is set, when SERVICER runs with MUNFLAG, or while
 * RADMODES shows remode, CDU zeroing, monitor or RR not in auto (bits 14,
 * 13, 11, 2).  Out of limits: set REPOSBIT, disable the tracker and the
 * error counters (channel 12 bits 14, 2), DORREPOS in 20 ms.
 **********************************************************************/
static void t4_rrgimon(void)
{
    agc_status_t st = AGC_STATUS_OK; /* WAITLIST result */

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return;
    }

    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_NORRMON) ||
        (agc_flag_test((agc_flag_id_t) AGC_FLAG_AVEGFLAG) &&
         agc_flag_test((agc_flag_id_t) AGC_FLAG_MUNFLAG)) ||
        ((agc_erase_radmodes() & OCT32002) != 0u))
    {
        return; /* TCF NORRGMON */
    }

    if (agc_rr_limchk(agc_erase_cdut(), agc_erase_cdus()))
    {
        return; /* angles in limits (RRLIMCHK L+3) */
    }

    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() + REPOSBIT));
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) OCT20002);
    st = agc_waitlist_call(DORREPOS_DELAY_CS, AGC_TASK_DORREPOS);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG) ||
        !AGC_ASSERT((agc_erase_radmodes() & REPOSBIT) != 0u))
    {
        return;
    }
}

/**********************************************************************
 * RRCDUCHK (T4RUPT_PROGRAM.agc pp.184-185) -- RR CDU fail monitor
 * (channel 30 bit 7)
 *
 * A change of the fail bit while the RR is in auto is copied into
 * RADMODES bit 7; a failure during P20/P22 (RNDVZFLG) is alarm 00515;
 * SETTRKF then updates the tracker fail lamp.
 *
 * RETURNS: true to go on to RRGIMON, false when the AGC goes straight to
 *          NORRGMON (bit changed with the RR not in auto)
 **********************************************************************/
static bool t4_rrcduchk(void)
{
    agc_status_t st  = AGC_STATUS_OK;                     /* SETTRKF   */
    uint16_t     chg = (uint16_t) ((agc_erase_radmodes() ^ /* changed  */
                                    agc_chan_read((int) AGC_CHAN_CHAN30)) &
                                   RCDUFBIT);

    if (chg == 0u)
    {
        return (true); /* BZF RRGIMON */
    }

    if ((agc_erase_radmodes() & AUTOMBIT) != 0u)
    {
        return (false); /* not in auto: TCF NORRGMON */
    }

    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() ^ RCDUFBIT));

    if (!AGC_ASSERT(((agc_erase_radmodes() ^
                      agc_chan_read((int) AGC_CHAN_CHAN30)) & RCDUFBIT) == 0u))
    {
        return (false);
    }

    if (((agc_erase_radmodes() & RCDUFBIT) == 0u) &&
        agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG))
    {
        agc_alarm(ALARM_RR_CDU_FAIL);
    }

    st = agc_rr_settrkf(); /* TRKFLCDU */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (false);
    }

    return (true);
}

/**********************************************************************
 * RRAUTCHK (T4RUPT_PROGRAM.agc pp.182-183) -- RR power-on-auto monitor
 * (channel 33 bit 2), then RRCDUCHK, RRGIMON and NORRGMON = GPMATRIX
 *
 * A change of the auto bit updates RADMODES and clears continuous
 * designate, remode, reposition, CDU zero and turn-on (MASK OCT05776).
 * Just switched on: set RRCDUZRO and TURNON and start RRTURNON in 10 ms,
 * skipping the two monitors this pass.  Switched off: disable the RR CDU
 * error counters (channel 12 bit 2, RRCDUCHK -3).
 *
 * RETURNS: AGC_STATUS_OK, or BAD_ARG when the waitlist or GPMATRIX fails
 **********************************************************************/
agc_status_t agc_t4_rr_monitor(void)
{
    agc_status_t st  = AGC_STATUS_OK;                     /* WAITLIST  */
    uint16_t     chg = (uint16_t) ((agc_erase_radmodes() ^ /* changed  */
                                    agc_chan_read((int) AGC_CHAN_CHAN33)) &
                                   AUTOMBIT);

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    if (chg != 0u)
    {
        /* LXCH RADMODES / RXOR LCHAN / MASK OCT05776 / TS RADMODES */
        agc_erase_set_radmodes(
            (uint16_t) ((agc_erase_radmodes() ^ chg) & OCT05776));

        if (!AGC_ASSERT(((agc_erase_radmodes() ^
                          agc_chan_read((int) AGC_CHAN_CHAN33)) &
                         AUTOMBIT) == 0u))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        if ((agc_erase_radmodes() & AUTOMBIT) == 0u)
        {
            /* just on: CA OCT10001 / ADS RADMODES */
            agc_erase_set_radmodes(
                (uint16_t) (agc_erase_radmodes() + OCT10001));
            st = agc_waitlist_call(RRTURNON_DELAY_CS, AGC_TASK_RRTURNON);

            if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            return (agc_t4_gimbal_matrix()); /* TCF NORRGMON */
        }

        ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) AUTOMBIT);
    }

    if (t4_rrcduchk())
    {
        t4_rrgimon();
    }

    return (agc_t4_gimbal_matrix());
} /* agc_t4_rr_monitor */
