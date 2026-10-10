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
#include "agc/agc_assert.h"
#include "agc/agc_state.h"
#include "agc/agc_channels.h"
#include "agc/agc_flags_lm.h"
#include "agc/agc_sysmisc.h"
#include "sysmisc_ext.h"

#define BIT2      000002u    /* AUTOMBIT: RR not in auto (inverted)        */
#define BIT7      000100u    /* RCDUFBIT: RR CDU OK                        */
#define BIT11     002000u    /* REPOSBIT                                   */
#define WORD15    077777u
#define OCT05776  005776u    /* clear cont. des., remode, repos, CDU zero, turn-on */
#define OCT10001  010001u    /* RRCDUZRO and TURNON bits                   */
#define OCT32002  032002u    /* remode, zeroing, monitor, not in auto      */
#define OCT20002  020002u    /* tracker enable and error counter enable    */
#define CDU_SIGN  040000u
#define RRTURNON_DELAY_CS 1u
#define DORREPOS_DELAY_CS 2u
#define ALARM_RR_CDU_FAIL ((agc_alarm_code_t)00515) /* RR CDU fail in P20/P22 */

static uint16_t radmodes;            /* RADMODES                           */
static agc_sp_t m11, m21, m22, m31, m32, cosmg;   /* scaled at 1           */

/* ---- accessors ---------------------------------------------------------- */
uint16_t agc_erase_radmodes(void)
{
    (void)AGC_ASSERT(radmodes <= WORD15);
    return radmodes;
}

void agc_erase_set_radmodes(uint16_t v)
{
    if (AGC_ASSERT(v <= WORD15)) {
        radmodes = v;
    }
}

agc_sp_t agc_erase_m11(void)   { return m11; }
agc_sp_t agc_erase_m21(void)   { return m21; }
agc_sp_t agc_erase_m22(void)   { return m22; }
agc_sp_t agc_erase_m31(void)   { return m31; }
agc_sp_t agc_erase_m32(void)   { return m32; }
agc_sp_t agc_erase_cosmg(void) { return cosmg; }

/* A CDU counter word as CAE reads it: ones' complement SP, scaled at pi
 * (half-revolutions), the SPSIN/SPCOS argument scaling.                   */
static agc_sp_t cdu_sp(uint16_t cdu)
{
    (void)AGC_ASSERT(cdu <= WORD15);
    int v = ((cdu & CDU_SIGN) != 0u) ? -(int)(WORD15 - cdu) : (int)cdu;
    (void)AGC_ASSERT(v >= AGC_SP_MIN && v <= AGC_SP_MAX);
    return (agc_sp_t)v;
}

/* EXTEND / MP: upper word of the SP x SP product.                         */
static agc_sp_t mp_hi(agc_sp_t a, agc_sp_t b)
{
    agc_dp_t p = agc_sp_mul(a, b);
    (void)AGC_ASSERT(p >= AGC_DP_MIN && p <= AGC_DP_MAX);
    return agc_dp_high(p);
}

/* GPMATRIX / DAPT4S (p.188): the SP matrix elements for the LM DAP.      */
agc_status_t agc_t4_gimbal_matrix(void)
{
    agc_sp_t mg = cdu_sp(agc_erase_cduz());
    agc_sp_t og = cdu_sp(agc_erase_cdux());
    m11 = agc_sp_sin(mg);                 /* SIN(MG)                       */
    cosmg = agc_sp_cos(mg);               /* COS(MG), only a factor        */
    m22 = agc_sp_sin(og);                 /* SIN(OG), also MR22            */
    m31 = mp_hi((agc_sp_t)-m22, cosmg);   /* -SIN(OG)COS(MG)               */
    m32 = agc_sp_cos(og);                 /* COS(OG), also MR23            */
    m21 = mp_hi(m32, cosmg);              /* COS(OG)COS(MG)                */
    (void)AGC_ASSERT(m11 >= AGC_SP_MIN && m21 >= AGC_SP_MIN);
    return AGC_STATUS_OK;                 /* ENDDAPT4 = RESUME             */
}

static void ch_clear(int chan, uint16_t bits)
{
    agc_status_t st = agc_chan_and(chan, (uint16_t)(~(uint32_t)bits & WORD15));
    (void)AGC_ASSERT(st == AGC_STATUS_OK);
    (void)AGC_ASSERT(bits <= WORD15);
}

/* RRGIMON / MONREPOS (p.186): RR gimbal limit monitor (R25).              */
static void t4_rrgimon(void)
{
    if (agc_flag_test((agc_flag_id_t)AGC_FLAG_NORRMON)) {
        return;                                   /* no angle monitor     */
    }
    if (agc_flag_test((agc_flag_id_t)AGC_FLAG_AVEGFLAG)
        && agc_flag_test((agc_flag_id_t)AGC_FLAG_MUNFLAG)) {
        return;                                   /* SERVICER + MUNFLAG   */
    }
    if ((radmodes & OCT32002) != 0u) {
        return;                                   /* remode, zeroing ...  */
    }
    if (agc_rr_limchk(agc_erase_cdut(), agc_erase_cdus())) {
        return;                                   /* angles in limits     */
    }
    radmodes = (uint16_t)(radmodes + BIT11);      /* reposition           */
    ch_clear((int)AGC_CHAN_CHAN12, (uint16_t)OCT20002);
    agc_status_t st = agc_waitlist_call(DORREPOS_DELAY_CS, AGC_TASK_DORREPOS);
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
}

/* RRCDUCHK (p.184): RR CDU fail (channel 30 bit 7).  Returns false when
 * the AGC goes straight to NORRGMON (RR not in auto).                     */
static bool t4_rrcduchk(void)
{
    uint16_t chg = (uint16_t)((radmodes ^ agc_chan_read((int)AGC_CHAN_CHAN30))
                              & BIT7);
    if (chg == 0u) {
        return true;                              /* on to RRGIMON         */
    }
    if ((radmodes & BIT2) != 0u) {
        return false;                             /* not in auto mode      */
    }
    radmodes = (uint16_t)(radmodes ^ BIT7);
    if ((radmodes & BIT7) == 0u
        && agc_flag_test((agc_flag_id_t)AGC_FLAG_RNDVZFLG)) {
        agc_status_t st = agc_alarm(ALARM_RR_CDU_FAIL);
        (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    }
    agc_status_t st = agc_rr_settrkf();           /* TRKFLCDU: lamp        */
    (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
    return true;
}

/* RRAUTCHK (p.182): RR power-on-auto (channel 33 bit 2), then RRCDUCHK,
 * RRGIMON and NORRGMON = GPMATRIX.                                        */
agc_status_t agc_t4_rr_monitor(void)
{
    uint16_t chg = (uint16_t)((radmodes ^ agc_chan_read((int)AGC_CHAN_CHAN33))
                              & BIT2);
    if (chg != 0u) {
        radmodes = (uint16_t)((radmodes ^ chg) & OCT05776);
        if ((radmodes & BIT2) == 0u) {            /* just on               */
            radmodes = (uint16_t)(radmodes + OCT10001);
            agc_status_t st = agc_waitlist_call(RRTURNON_DELAY_CS,
                                                AGC_TASK_RRTURNON);
            (void)AGC_ASSERT(st != AGC_STATUS_BAD_ARG);
            return agc_t4_gimbal_matrix();
        }
        ch_clear((int)AGC_CHAN_CHAN12, (uint16_t)BIT2);  /* RRCDUCHK -3    */
    }
    if (t4_rrcduchk()) {
        t4_rrgimon();
    }
    return agc_t4_gimbal_matrix();
}
