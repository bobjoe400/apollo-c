/**********************************************************************
 * FILE NAME: service.c
 *
 * PURPOSE:
 *   Small shared service routines: flag set/clear (UPFLAG, DOWNFLAG,
 *   UPENT2, DOWNENT2), job delay (DELAYJOB, WAKER), block transfer
 *   (GENTRAN), B5OFF and the CM tracker lamp routines.
 *
 * AGC SOURCE:
 *   Comanche055 SERVICE_ROUTINES.agc pp.1485-1492
 *   Luminary099 SERVICE_ROUTINES.agc pp.1374-1380 (differs: UPENT2 /
 *   DOWNENT2 always on FLAGWRD4, DELAYJOB bails out through BAILOUT1, no
 *   TRFAILOF / TRFAILON)
 *
 * EXTERNAL VARIABLES:
 *   (AGC erasables reached through accessors)
 *   Name         Type      I/O  Scale  Description
 *   -----------  --------  ---  -----  ----------------------------------
 *   FLAGWRD0..   uint16_t  I/O  bits   flagwords (agc_flagword_get/set)
 *   EXTVBACT     uint16_t  I/O  bits   extended verb activity
 *   DSPTAB +11D  uint16_t  I/O  bits   lamp relay word
 *   OPTMODES     uint16_t  I/O  bits   CM optics modes (TRFAILOF)
 *
 * EXTERNAL REFERENCES:
 *   Name                Description
 *   ------------------  ---------------------------------------------
 *   agc_waitlist_call   WAITLIST (DELAYJOB -> WAKER)
 *   agc_exec_job_sleep  JOBSLEEP / JOBWAKE
 *   agc_bailout         BAILOUT 01104
 *
 * ABNORMAL TERMINATION CONDITIONS, ALARMS:
 *   BAILOUT 01104 when all four DELAYLOCs are in use.
 *
 * ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS:
 *   Both vehicles (TRFAILOF / TRFAILON CSM only).
 *
 * NOTES:
 *   Vehicle differences: UPENT2/DOWNENT2 take "flagword in bits 1-3,
 *   bits in 4-15" on the CSM and always work on FLAGWRD4 on the LM; the
 *   DELAYJOB bailout is the same code (1104) by a different entry
 *   (BAILOUT1 on the LM); TRFAILOF/TRFAILON exist only in Comanche.
 **********************************************************************/
#include <stddef.h>

#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/io/service.h"

#define WORD15            077777u                    /* one AGC word, 15 bits */
#define OCT7              000007u                    /* OCT7: flagword number (CSM) */
#define OCT77770          077770u                    /* OCT77770: flag bits */
#define OCT40200          040200u                    /* lamp word send bit + TRACKER (bit 8) */
#define BIT5              000020u                    /* BIT5 */
#define BIT7              000100u                    /* BIT7 */
#define BIT15             040000u                    /* BIT15: lamp word send bit */
#define LAMPWORD          11                         /* DSPTAB +11D */
#define DELAYNUM          4                          /* DELAYLOC +0..+3                            */
#define LM_ENT2_FLAGWORD  4                          /* LM UPENT2/DOWNENT2 work on FLAGWRD4        */
#define ALARM_NO_DELAYLOC ((agc_alarm_code_t) 01104) /* no DELAYLOC free */

/* DELAYLOC +0..+3: the job sleeping in each slot (AGC: its FCADR) and,
 * in C, the time it is due (the waitlist runs WAKER in due order).        */
static agc_job_id_t delayloc[DELAYNUM]; /* DELAYLOC +0..+3: sleeping job ids */
static uint32_t     delaydue[DELAYNUM]; /* cs, 2^28 wrap                  */

/**********************************************************************
 * UPFLAG (p.1486): set a named flag; returns the new flagword.
 *
 * ARGUMENTS:
 *   f  agc_flag_id_t  I   the flag (flagword number and bit, the AGC's
 *                         ADRES word after the TC)
 **********************************************************************/
uint16_t agc_svc_upflag(agc_flag_id_t f)
{
    agc_inhint();
    agc_flag_set(f);
    agc_relint();

    return (agc_flagword_get(agc_flag_word(f)));
}

/**********************************************************************
 * DOWNFLAG (p.1486): clear a named flag; returns the new flagword.
 *
 * ARGUMENTS:
 *   f  agc_flag_id_t  I   the flag (flagword number and bit)
 **********************************************************************/
uint16_t agc_svc_downflag(agc_flag_id_t f)
{
    agc_inhint();
    agc_flag_clear(f);
    agc_relint();

    return (agc_flagword_get(agc_flag_word(f)));
}

/**********************************************************************
 * AGC: none (the TS L / MASK OCT7 / XCH L / MASK OCT77770 head of UPENT2
 * and DOWNENT2, SERVICE_ROUTINES.agc p.1485) -- split the argument
 *
 * ARGUMENTS:
 *   arg   uint16_t    I   CSM: flagword number in bits 1-3, flag bits in
 *                         4-15; LM: flag bits in 4-15 (FLAGWRD4)
 *   bits  uint16_t *  O   the flag bits (arg & 77770)
 *
 * RETURNS: the flagword index (LM 4, CSM 0..7)
 **********************************************************************/
static int ent2_word(uint16_t arg, uint16_t *bits)
{
    *bits    = (uint16_t) (arg & OCT77770);
    int word = (agc_state_vehicle() == AGC_VEHICLE_LM) ? LM_ENT2_FLAGWORD : (int) (arg & OCT7); /* LM: FLAGWRD4 */

    return (word);
}

/**********************************************************************
 * UPENT2 (C p.1485 / L p.1374): OR bits into a flagword (display
 * interface flags).
 *
 * ARGUMENTS:
 *   arg  uint16_t  I   flagword number (CSM, bits 1-3) and the bits to
 *                      set (4-15)
 *
 * (Returns to L+2.)
 **********************************************************************/
void agc_svc_upent2(uint16_t arg)
{
    uint16_t bits = 0;                     /* bits 4-15 of the argument */
    int      word = ent2_word(arg, &bits); /* flagword index */

    agc_inhint();
    agc_flagword_set(word, (uint16_t) (agc_flagword_get(word) | bits));
    agc_relint();
}

/**********************************************************************
 * DOWNENT2 (C p.1485 / L p.1374): clear bits of a flagword.
 *
 * ARGUMENTS:
 *   arg  uint16_t  I   flagword number (CSM, bits 1-3) and the bits to
 *                      clear (4-15)
 *
 * (Returns to L+2.)
 **********************************************************************/
void agc_svc_downent2(uint16_t arg)
{
    uint16_t bits = 0;                     /* bits 4-15 of the argument */
    int      word = ent2_word(arg, &bits); /* flagword index */

    agc_inhint();
    agc_flagword_set(word, (uint16_t) (agc_flagword_get(word) & ~(uint32_t) bits & WORD15));
    agc_relint();
}

/**********************************************************************
 * DELAYJOB (p.1488): put the calling job to sleep for delay_cs and wake
 * it as wake_as (the AGC wakes it at the caller's return address).  The
 * job must return after an OK.  BAILOUT 1104 if all four DELAYLOCs are
 * in use.
 **********************************************************************/
agc_status_t agc_svc_delayjob(uint16_t delay_cs, agc_job_id_t wake_as)
{
    agc_inhint();
    int slot = -1; /* free DELAYLOC (DELLOOP) */

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

        return (agc_bailout(ALARM_NO_DELAYLOC)); /* restart requested */
    }

    delayloc[slot]  = wake_as;
    delaydue[slot]  = (agc_time_now_cs() + delay_cs) & (AGC_TIME_MODULUS_CS - 1u);
    agc_status_t st = agc_waitlist_call(delay_cs, AGC_TASK_WAKER); /* WAITLIST result */

    agc_relint();

    if (st != AGC_STATUS_OK)
    {
        delayloc[slot] = AGC_JOB_NONE;

        return (st);
    }

    agc_exec_job_sleep(wake_as);

    return (AGC_STATUS_OK); /* TC JOBSLEEP            */
} /* agc_svc_delayjob */

/**********************************************************************
 * AGC: none (helper for WAKER, SERVICE_ROUTINES.agc p.1488) -- how late a
 * DELAYLOC's job is
 *
 * ARGUMENTS:
 *   i    int       I   DELAYLOC index, 0..3
 *   now  uint32_t  I   TIME2,TIME1, cs
 *
 * RETURNS: cs since the slot came due (negative while it is still to
 *          come)
 **********************************************************************/
static int32_t delay_lateness(int i, uint32_t now)
{
    uint32_t d    = (now - delaydue[i]) & (AGC_TIME_MODULUS_CS - 1u);                                              /* cs since due, 2^28 wrap */
    int32_t  late = (d >= (AGC_TIME_MODULUS_CS >> 1)) ? (int32_t) d - (int32_t) AGC_TIME_MODULUS_CS : (int32_t) d; /* signed lateness, cs */

    return (late);
}

/**********************************************************************
 * WAKER (p.1488): free a DELAYLOC and JOBWAKE its job.  slot -1 (the task
 * entry) picks the earliest-due occupied slot, the one this WAKER task was
 * scheduled for (the AGC passes the slot in BBANK).
 *
 * ARGUMENTS:
 *   slot  int  I   DELAYLOC index 0..3 (the AGC passes it in BBANK), or
 *                  -1 from the WAKER task: the earliest due
 *
 * RETURNS: the JOBWAKE status, NOT_FOUND when no DELAYLOC is in use,
 *          BAD_ARG on a bad slot
 **********************************************************************/
agc_status_t agc_svc_delay_wake(int slot)
{
    uint32_t now = agc_time_now_cs(); /* TIME2,TIME1, cs */

    for (int i = 0; i < DELAYNUM && slot < 0; i++)
    {
        if (delayloc[i] == AGC_JOB_NONE)
        {
            continue;
        }

        bool earliest = true; /* slot i is the earliest due */

        for (int j = 0; j < DELAYNUM; j++)
        {
            if (j != i && delayloc[j] != AGC_JOB_NONE && delay_lateness(j, now) > delay_lateness(i, now))
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

    agc_job_id_t job = delayloc[slot]; /* job in the slot */

    delayloc[slot] = AGC_JOB_NONE; /* make DELAYLOC available */

    return (agc_exec_job_wake(job));
} /* agc_svc_delay_wake */

/**********************************************************************
 * STARTSB2 (FRESH_START_AND_RESTART.agc p.184 CM) -- DELAYLOC +0..+3 = 0
 *
 * ARGUMENTS: none
 **********************************************************************/
void agc_svc_startsub_reset(void)
{
    for (int i = 0; i < DELAYNUM; i++)
    {
        delayloc[i] = AGC_JOB_NONE;
        delaydue[i] = 0;
    }
}

/**********************************************************************
 * GENTRAN (p.1490): block transfer of n words, from the top down.
 *
 * ARGUMENTS:
 *   from  const uint16_t *  I   words L .. L+N-1 (ADRES L after the TC)
 *   to    uint16_t *        O   words M .. M+N-1 (ADRES M)
 *   n     int               I   N, 1..256 (the AGC passes N-1 in A)
 *
 * (Returns to Q+2.)
 **********************************************************************/
void agc_svc_gentran(const uint16_t *from, uint16_t *to, int n)
{
    agc_inhint(); /* no RELINT (MOD 2)      */

    for (int i = n - 1; i >= 0; i--)
    {
        to[i] = from[i];
    }
}

/**********************************************************************
 * B5OFF (p.1491): clear EXTVBACT bit 5 (set by TESTXACT); the caller then
 * ends its job.
 **********************************************************************/
void agc_svc_b5off(void)
{
    uint16_t x = agc_erase_extvbact(); /* EXTVBACT bits */

    agc_erase_set_extvbact((uint16_t) (x & ~(uint32_t) BIT5));
}

/**********************************************************************
 * TRFAILOF (C p.1492): TRACKER lamp off (DSPTAB +11D bit 8) and re-arm
 * the OCDU fail lamp (OPTMODES bit 7).
 **********************************************************************/
void agc_svc_trfailof(void)
{
    agc_inhint();
    uint16_t w = agc_erase_dsptab(LAMPWORD); /* DSPTAB +11D */

    agc_erase_set_dsptab(LAMPWORD, (uint16_t) ((w & ~(uint32_t) OCT40200) + BIT15));
    agc_erase_set_optmodes((uint16_t) (agc_erase_optmodes() | BIT7));
    agc_relint();
}

/**********************************************************************
 * TRFAILON (C p.1492): TRACKER lamp on.
 **********************************************************************/
void agc_svc_trfailon(void)
{
    agc_inhint();
    uint16_t w = agc_erase_dsptab(LAMPWORD); /* DSPTAB +11D */

    agc_erase_set_dsptab(LAMPWORD, (uint16_t) (w | OCT40200));
    agc_relint();
}
