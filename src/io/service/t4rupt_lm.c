/*
 * t4rupt_lm.c -- Luminary099 T4RUPT_PROGRAM.agc, LM part:
 * RRAUTCHK (p.182), RRCDUCHK (p.184), RRGIMON (p.186) -- the rendezvous
 * radar inbit and gimbal-limit monitors -- and GPMATRIX / DAPT4S (p.188),
 * the gimbal-to-pilot axis matrix for the LM DAP.
 *
 * T4JUMP runs RRAUTCHK at phases 1, 5 (falling into GPMATRIX through
 * NORRGMON) and DAPT4S at phases 3, 7, so GPMATRIX runs 4 times a second.
 *
 * Erasables owned here: RADMODES (bit word, shared with prog-rendz and
 * pinball through the accessors below) and the DAP matrix elements M11,
 * M21, M22, M31, M32 and COSMG (SP, scaled at 1):
 *
 *         | sin MG            0        1 |
 *   M   = | cos MG cos OG     sin OG   0 |     (AOG = CDUX, AMG = CDUZ)
 *    GP   | -cos MG sin OG    cos OG   0 |
 *
 * The RR service routines RRTURNON, DORREPOS, RRLIMCHK and SETTRKF are in
 * P20-P25.agc (prog-rendz); they are called through sysmisc_ext.h.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/flags_lm.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"
#include "agc/programs/rendezvous.h"

#define BIT2   000002u /* AUTOMBIT: RR not in auto (inverted)        */
#define BIT7   000100u /* RCDUFBIT: RR CDU OK                        */
#define BIT11  002000u /* REPOSBIT                                   */
#define WORD15 077777u
#define OCT05776 \
    005776u /* clear cont. des., remode, repos, CDU zero, turn-on */
#define OCT10001          010001u /* RRCDUZRO and TURNON bits                   */
#define OCT32002          032002u /* remode, zeroing, monitor, not in auto      */
#define OCT20002          020002u /* tracker enable and error counter enable    */
#define CDU_SIGN          040000u
#define RRTURNON_DELAY_CS 1u
#define DORREPOS_DELAY_CS 2u
#define ALARM_RR_CDU_FAIL \
    ((agc_alarm_code_t) 00515) /* RR CDU fail in P20/P22 */

static agc_sp_t m11, m21, m22, m31, m32, cosmg; /* scaled at 1           */

/* ---- accessors ---------------------------------------------------------- */
/* RADMODES EQUALS FLGWRD12 (FLAGWORD_ASSIGNMENTS p.85): one storage word,
 * the flagword; these name it for the radar code.                        */
uint16_t agc_erase_radmodes(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (0); /* no valid result */
    }

    return (agc_flagword_get(12));
}

void agc_erase_set_radmodes(uint16_t v)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return;
    }

    if (AGC_ASSERT(v <= WORD15))
    {
        agc_flagword_set(12, v);
    }
}

agc_sp_t agc_erase_m11(void)
{
    return (m11);
}

agc_sp_t agc_erase_m21(void)
{
    return (m21);
}

agc_sp_t agc_erase_m22(void)
{
    return (m22);
}

agc_sp_t agc_erase_m31(void)
{
    return (m31);
}

agc_sp_t agc_erase_m32(void)
{
    return (m32);
}

agc_sp_t agc_erase_cosmg(void)
{
    return (cosmg);
}

/* A CDU counter word as CAE reads it: ones' complement SP, scaled at pi
 * (half-revolutions), the SPSIN/SPCOS argument scaling.                   */
static agc_sp_t cdu_sp(uint16_t cdu)
{
    if (!AGC_ASSERT(cdu <= WORD15))
    {
        return (0); /* no valid result */
    }

    int v = ((cdu & CDU_SIGN) != 0u) ? -(int) (WORD15 - cdu) : (int) cdu;

    if (!AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX))
    {
        return (0); /* no valid result */
    }

    return ((agc_sp_t) v);
}

/* EXTEND / MP: upper word of the SP x SP product.                         */
static agc_sp_t mp_hi(agc_sp_t a, agc_sp_t b)
{
    agc_dp_t p = agc_sp_mul(a, b);

    if (!AGC_ASSERT(p >= AGC_DP_MIN && p <= AGC_DP_MAX))
    {
        return (0); /* no valid result */
    }

    return (agc_dp_high(p));
}

/* GPMATRIX / DAPT4S (p.188): the SP matrix elements for the LM DAP.      */
agc_status_t agc_t4_gimbal_matrix(void)
{
    agc_sp_t mg = cdu_sp(agc_erase_cduz());
    agc_sp_t og = cdu_sp(agc_erase_cdux());

    m11   = agc_sp_sin(mg);        /* SIN(MG)                       */
    cosmg = agc_sp_cos(mg);        /* COS(MG), only a factor        */
    m22   = agc_sp_sin(og);        /* SIN(OG), also MR22            */
    m31   = mp_hi((agc_sp_t) -m22, cosmg); /* -SIN(OG)COS(MG)               */
    m32   = agc_sp_cos(og);              /* COS(OG), also MR23            */
    m21   = mp_hi(m32, cosmg);           /* COS(OG)COS(MG)                */

    if (!AGC_ASSERT(m11 >= AGC_SP_MIN && m21 >= AGC_SP_MIN))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK); /* ENDDAPT4 = RESUME             */
}

static void ch_clear(int chan, uint16_t bits)
{
    agc_chan_and(chan, (uint16_t) (~(uint32_t) bits & WORD15));

    if (!AGC_ASSERT(bits <= WORD15))
    {
        return;
    }
}

/* RRGIMON / MONREPOS (p.186): RR gimbal limit monitor (R25).              */
static void t4_rrgimon(void)
{
    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_NORRMON))
    {
        return; /* no angle monitor     */
    }

    if (agc_flag_test((agc_flag_id_t) AGC_FLAG_AVEGFLAG) &&
        agc_flag_test((agc_flag_id_t) AGC_FLAG_MUNFLAG))
    {
        return; /* SERVICER + MUNFLAG   */
    }

    if ((agc_erase_radmodes() & OCT32002) != 0u)
    {
        return; /* remode, zeroing ...  */
    }

    if (agc_rr_limchk(agc_erase_cdut(), agc_erase_cdus()))
    {
        return; /* angles in limits     */
    }

    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() + BIT11)); /* reposition           */
    ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) OCT20002);
    agc_status_t st = agc_waitlist_call(DORREPOS_DELAY_CS, AGC_TASK_DORREPOS);

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return;
    }
}

/* RRCDUCHK (p.184): RR CDU fail (channel 30 bit 7).  Returns false when
 * the AGC goes straight to NORRGMON (RR not in auto).                     */
static bool t4_rrcduchk(void)
{
    uint16_t chg =
        (uint16_t) ((agc_erase_radmodes() ^
                     agc_chan_read((int) AGC_CHAN_CHAN30)) & BIT7);

    if (chg == 0u)
    {
        return (true); /* on to RRGIMON         */
    }

    if ((agc_erase_radmodes() & BIT2) != 0u)
    {
        return (false); /* not in auto mode      */
    }

    agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() ^ BIT7));

    if ((agc_erase_radmodes() & BIT7) == 0u &&
        agc_flag_test((agc_flag_id_t) AGC_FLAG_RNDVZFLG))
    {
        agc_alarm(ALARM_RR_CDU_FAIL);
    }

    agc_status_t st = agc_rr_settrkf(); /* TRKFLCDU: lamp        */

    if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
    {
        return (false);
    }

    return (true);
}

/* RRAUTCHK (p.182): RR power-on-auto (channel 33 bit 2), then RRCDUCHK,
 * RRGIMON and NORRGMON = GPMATRIX.                                        */
agc_status_t agc_t4_rr_monitor(void)
{
    uint16_t chg =
        (uint16_t) ((agc_erase_radmodes() ^
                     agc_chan_read((int) AGC_CHAN_CHAN33)) & BIT2);

    if (chg != 0u)
    {
        agc_erase_set_radmodes((uint16_t) ((agc_erase_radmodes() ^ chg) &
                                           OCT05776));

        if ((agc_erase_radmodes() & BIT2) == 0u)
        { /* just on               */
            agc_erase_set_radmodes((uint16_t) (agc_erase_radmodes() +
                                               OCT10001));
            agc_status_t st =
                agc_waitlist_call(RRTURNON_DELAY_CS, AGC_TASK_RRTURNON);

            if (!AGC_ASSERT(st != AGC_STATUS_BAD_ARG))
            {
                return (AGC_STATUS_BAD_ARG);
            }

            return (agc_t4_gimbal_matrix());
        }

        ch_clear((int) AGC_CHAN_CHAN12, (uint16_t) BIT2); /* RRCDUCHK -3    */
    }

    if (t4_rrcduchk())
    {
        t4_rrgimon();
    }

    return (agc_t4_gimbal_matrix());
}
