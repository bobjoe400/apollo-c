/*
 * service.c -- SERVICE_ROUTINES.agc: small shared routines.
 *
 * Sources: Comanche055 SERVICE_ROUTINES.agc pp. 1485-1492,
 *          Luminary099 SERVICE_ROUTINES.agc pp. 1374-1380.
 *
 * Vehicle differences: UPENT2/DOWNENT2 take "flagword in bits 1-3, bits in
 * 4-15" on the CSM and always work on FLAGWRD4 on the LM; the DELAYJOB
 * bailout is the same code (1104) by a different entry (BAILOUT1 on the
 * LM); TRFAILOF/TRFAILON exist only in Comanche.
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service_internal.h"
#include "sysmisc_ext.h"

#define WORD15            077777u
#define OCT7              000007u
#define OCT77770          077770u
#define OCT40200          040200u /* lamp word change bit + TRACKER (bit 8)    */
#define BIT5              000020u
#define BIT7              000100u
#define BIT15             040000u
#define LAMPWORD          11
#define DELAYNUM          4 /* DELAYLOC +0..+3                            */
#define LM_ENT2_FLAGWORD  4 /* LM UPENT2/DOWNENT2 work on FLAGWRD4        */
#define ALARM_NO_DELAYLOC ((agc_alarm_code_t) 01104)

/* DELAYLOC +0..+3: the job sleeping in each slot (AGC: its FCADR) and,
 * in C, the time it is due (the waitlist runs WAKER in due order).        */
static agc_job_id_t delayloc[DELAYNUM];
static uint32_t     delaydue[DELAYNUM]; /* cs, 2^28 wrap                  */

/* UPFLAG (p.1486): set a named flag; returns the new flagword.           */
uint16_t agc_svc_upflag(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return (0);
    }

    agc_inhint();
    agc_flag_set(f);
    agc_relint();

    return (agc_flagword_get(agc_flag_word(f)));
}

/* DOWNFLAG (p.1486): clear a named flag; returns the new flagword.       */
uint16_t agc_svc_downflag(agc_flag_id_t f)
{
    if (!AGC_ASSERT(agc_flag_valid(f)))
    {
        return (0);
    }

    agc_inhint();
    agc_flag_clear(f);
    agc_relint();

    return (agc_flagword_get(agc_flag_word(f)));
}

/* Split the UPENT2/DOWNENT2 argument: CSM flagword in bits 1-3, LM 4.    */
static int ent2_word(uint16_t arg, uint16_t *bits)
{
    if (!AGC_ASSERT(bits != NULL))
    {
        return (-1);
    }

    *bits = (uint16_t) (arg & OCT77770);
    int word = (agc_state_vehicle() == AGC_VEHICLE_LM) ? LM_ENT2_FLAGWORD :
               (int) (arg & OCT7);

    if (!AGC_ASSERT(word >= 0 && word < AGC_FLAGWORDS))
    {
        return (-1);
    }

    return (word);
}

/* UPENT2 (C p.1485 / L p.1374): OR bits into a flagword (display
 * interface flags).                                                       */
agc_status_t agc_svc_upent2(uint16_t arg)
{
    uint16_t bits = 0;
    int      word = ent2_word(arg, &bits);

    if (!AGC_ASSERT(word >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_flagword_set(word, (uint16_t) (agc_flagword_get(word) | bits));
    agc_relint();

    return (AGC_STATUS_OK);
}

/* DOWNENT2 (C p.1485 / L p.1374): clear bits of a flagword.              */
agc_status_t agc_svc_downent2(uint16_t arg)
{
    uint16_t bits = 0;
    int      word = ent2_word(arg, &bits);

    if (!AGC_ASSERT(word >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    agc_flagword_set(
        word, (uint16_t) (agc_flagword_get(word) & ~(uint32_t) bits & WORD15));
    agc_relint();

    return (AGC_STATUS_OK);
}

/* DELAYJOB (p.1488): put the calling job to sleep for delay_cs and wake
 * it as wake_as (the AGC wakes it at the caller's return address).  The
 * job must return after an OK.  BAILOUT 1104 if all four DELAYLOCs are
 * in use.                                                                 */
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as)
{
    if (!AGC_ASSERT(wake_as != AGC_JOB_NONE) ||
        !AGC_ASSERT(delay_cs > 0u && delay_cs <= AGC_WAITLIST_MAX_DELAY))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    int slot = -1;

    for (int i = DELAYNUM - 1; i >= 0 && slot < 0; i--)
    { /* DELLOOP     */
        if (delayloc[i] == AGC_JOB_NONE)
        {
            slot = i;
        }
    }

    if (slot < 0)
    {
        agc_relint();
        agc_status_t bst =
            agc_bailout(ALARM_NO_DELAYLOC); /* restart requested */

        if (!AGC_ASSERT(bst == AGC_STATUS_BAILOUT))
        {
            return (AGC_STATUS_BAD_ARG);
        }

        return (bst);
    }

    delayloc[slot] = wake_as;
    delaydue[slot] =
        (agc_time_now_cs() + delay_cs) & (AGC_TIME_MODULUS_CS - 1u);
    agc_status_t st = agc_waitlist_call(delay_cs, AGC_TASK_WAKER);

    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        delayloc[slot] = AGC_JOB_NONE;

        return (st);
    }

    agc_exec_job_sleep(wake_as);

    return (AGC_STATUS_OK); /* TC JOBSLEEP            */
}

/* Lateness of a DELAYLOC's due time, as a signed count (2^28 wrap).    */
static int32_t delay_lateness(int i, uint32_t now)
{
    if (!AGC_ASSERT(i >= 0 && i < DELAYNUM))
    {
        return (0); /* no valid result */
    }

    uint32_t d    = (now - delaydue[i]) & (AGC_TIME_MODULUS_CS - 1u);
    int32_t  late = (d >= (AGC_TIME_MODULUS_CS >> 1)) ?
                    (int32_t) d - (int32_t) AGC_TIME_MODULUS_CS :
                    (int32_t) d;

    if (!AGC_ASSERT(late > -(int32_t) AGC_TIME_MODULUS_CS))
    {
        return (0); /* no valid result */
    }

    return (late);
}

/* WAKER (p.1488): free a DELAYLOC and JOBWAKE its job.  slot -1 (the task
 * entry) picks the earliest-due occupied slot, the one this WAKER task was
 * scheduled for (the AGC passes the slot in BBANK).                       */
agc_status_t agc_svc_delay_wake(int slot)
{
    if (!AGC_ASSERT(slot >= -1 && slot < DELAYNUM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    uint32_t now = agc_time_now_cs();

    for (int i = 0; i < DELAYNUM && slot < 0; i++)
    {
        if (delayloc[i] == AGC_JOB_NONE)
        {
            continue;
        }

        bool earliest = true;

        for (int j = 0; j < DELAYNUM; j++)
        {
            if (j != i && delayloc[j] != AGC_JOB_NONE &&
                delay_lateness(j, now) > delay_lateness(i, now))
            {
                earliest = false;
            }
        }

        if (earliest)
        {
            slot = i;
        }
    }

    if (slot < 0 || delayloc[slot] == AGC_JOB_NONE)
    {
        return (AGC_STATUS_NOT_FOUND);
    }

    agc_job_id_t job = delayloc[slot];

    delayloc[slot] = AGC_JOB_NONE; /* make DELAYLOC available */

    return (agc_exec_job_wake(job));
}

/* STARTSB2 (FRESH_START): DELAYLOC +0..+3 = 0.                           */
agc_status_t agc_svc_startsub_reset(void)
{
    for (int i = 0; i < DELAYNUM; i++)
    {
        delayloc[i] = AGC_JOB_NONE;
        delaydue[i] = 0;
    }

    if (!AGC_ASSERT(delayloc[0] == AGC_JOB_NONE))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_OK);
}

/* GENTRAN (p.1490): block transfer of n words, from the top down.        */
agc_status_t agc_svc_gentran(const uint16_t *from, uint16_t *to, int n)
{
    if (!AGC_ASSERT(from != NULL && to != NULL) ||
        !AGC_ASSERT(n >= 1 && n <= 0400))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint(); /* no RELINT (MOD 2)      */

    for (int i = n - 1; i >= 0; i--)
    {
        to[i] = from[i];
    }

    return (AGC_STATUS_OK);
}

/* B5OFF (p.1491): clear EXTVBACT bit 5 (set by TESTXACT); the caller then
 * ends its job.                                                           */
agc_status_t agc_svc_b5off(void)
{
    uint16_t x = agc_erase_extvbact();

    if (!AGC_ASSERT(x <= WORD15))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_erase_set_extvbact((uint16_t) (x & ~(uint32_t) BIT5));

    return (AGC_STATUS_OK);
}

/* TRFAILOF (C p.1492): TRACKER lamp off (DSPTAB +11D bit 8) and re-arm
 * the OCDU fail lamp (OPTMODES bit 7).                                    */
agc_status_t agc_svc_trfailof(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    uint16_t w = agc_erase_dsptab(LAMPWORD);

    agc_erase_set_dsptab(
        LAMPWORD, (uint16_t) ((w & ~(uint32_t) OCT40200) + BIT15));
    agc_erase_set_optmodes((uint16_t) (agc_erase_optmodes() | BIT7));
    agc_relint();

    return (AGC_STATUS_OK);
}

/* TRFAILON (C p.1492): TRACKER lamp on.                                   */
agc_status_t agc_svc_trfailon(void)
{
    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_CSM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    uint16_t w = agc_erase_dsptab(LAMPWORD);

    agc_erase_set_dsptab(LAMPWORD, (uint16_t) (w | OCT40200));
    agc_relint();

    return (AGC_STATUS_OK);
}
