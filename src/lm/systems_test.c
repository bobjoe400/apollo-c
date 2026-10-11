/*
 * systems_test.c -- P07 SYSTEMS TEST, Luminary099 (V92 SYSTEST -> REDO):
 * SYSTEM_TEST_STANDARD_LEAD_INS.agc (pp. 370-372), IMU_PERFORMANCE_TEST_2
 * (pp. 373-381) and IMU_PERFORMANCE_TESTS_4 (pp. 382-389).
 *
 * Translated: the subroutines other code can use -- ZEROING (zero a block
 * of erasable), FINETIME (read the 1/1600 s scalar, channels 3 and 4,
 * without catching it mid-carry), OVERFFIX (fix a negative time
 * difference across a TIME2 overflow).
 *
 * STUB: REDO and the IMU performance tests behind it (GEOIMUTT, the
 * position/drift tests, ESTIMS, the gyro torquing and PIP pulse catching
 * loops) are a ground-test program of about 700 lines that drives the IMU
 * through the test positions.  The rope does not hold the whole test: the
 * drift filter's calculation (PERFERAS, p.387) does "TC ATIGINC -- GOTO
 * ERASABLE TO CALCULATE", into an erasable-memory program that "MUST BE
 * LOADED BEFORE ANY ATTEMPT IS MADE TO RUN THE IMU PERFORMANCE TEST".
 * That program is not in Luminary099, so the test cannot be translated
 * whole; agc_job_systems_test sets MM 07 and returns
 * AGC_STATUS_UNIMPLEMENTED (see docs/modules/sys-misc.md "Stubs").  The
 * earth-rate routines ERTHRVSE / EARTHR (p.380) are the same text as the
 * CM prelaunch ones (src/csm/prelaunch_align.c, agc_csm_prelaunch_*).
 *
 * The erasable-memory program helpers in SYSTEM_TEST_STANDARD_LEAD_INS
 * (E/BKCALL, E/CALL, E/SWITCH, E/JOBWAK) exist only so that programs
 * loaded into erasable memory can make bank calls; they vanish in C like
 * the inter-bank routines (src/io/service/interbank.c).
 */
#include <stddef.h>

#include "agc/core/assert.h"
#include "agc/core/channels.h"
#include "agc/core/interp.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"
#include "agc/imu/imu.h"
#include "agc/io/service.h"

#define POSMAX            037777u
#define WORD15            077777u
#define ZEROING_MAX       0400 /* longest block cleared (one EBANK)     */
#define FINETIME_TRIES    3    /* READ, re-READ, and the POSMAX retry   */
#define SYSTEMS_TEST_MODE 07   /* NEWMODEX, MM 07                        */
#define ONEDPP            ((agc_dp_t) 1)

/* ZEROING (IMU_PERFORMANCE_TEST_2 p.379): store +0 in n consecutive words
 * (the AGC takes the address in A and the count - 1 in ZERONDX1).         */
agc_status_t agc_systems_test_zeroing(uint16_t *block, int n)
{
    if (!AGC_ASSERT(block != NULL) || !AGC_ASSERT(n >= 1 && n <= ZEROING_MAX))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    for (int i = 0; i < n; i++)
    {
        block[i] = 0;
    }

    return (AGC_STATUS_OK);
}

/* FINETIME (p.380): LOSCALAR read twice so a carry between the reads is
 * seen; if it is POSMAX (about to carry into HISCALAR) read again.  Returns
 * with interrupts inhibited, like the AGC.  *t = HISCALAR, LOSCALAR as a DP
 * count of the scalar (1/1600 s per LOSCALAR count, scale as channel 3/4). */
agc_status_t agc_systems_test_finetime(agc_dp_t *t)
{
    if (!AGC_ASSERT(t != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_inhint();
    uint16_t lo = POSMAX;

    for (int k = 0; k < FINETIME_TRIES && lo == POSMAX; k++)
    {
        lo             = agc_chan_read((int) AGC_CHAN_LOSCALAR);
        uint16_t again = agc_chan_read((int) AGC_CHAN_LOSCALAR);

        if (again != lo)
        { /* RXOR, BZF               */
            lo = agc_chan_read((int) AGC_CHAN_LOSCALAR);
        }
    }

    uint16_t hi = agc_chan_read((int) AGC_CHAN_HISCALAR);

    if (!AGC_ASSERT(hi <= WORD15 && lo <= WORD15))
    {
        agc_relint();

        return (AGC_STATUS_BAD_ARG);
    }

    *t = agc_dp_pack((agc_sp_t) (hi & POSMAX), (agc_sp_t) (lo & POSMAX));

    return ((lo == POSMAX) ? AGC_STATUS_BUSY : AGC_STATUS_OK);
}

/* OVERFFIX (p.378): DAD DPPOSMAX, DAD ONEDPP -- add 2^28 to a time
 * difference that went negative across TIME2 overflow (wraps like DAD).   */
agc_dp_t agc_systems_test_overffix(agc_interp_t *ip, agc_dp_t a)
{
    if (!AGC_ASSERT(ip != NULL))
    {
        return (a);
    }

    agc_dp_t s = agc_dp_add(ip, a, AGC_DP_MAX);

    s = agc_dp_add(ip, s, ONEDPP);

    if (!AGC_ASSERT(s >= AGC_DP_MIN && s <= AGC_DP_MAX))
    {
        return (a);
    }

    return (s);
}

/* REDO (V92, LM p.373 / Comanche SYSTEM_TEST_STANDARD_LEAD_INS): NEWMODEX
 * 07, then the IMU performance tests.  Comanche falls into IMUTEST (imu
 * lane, prelaunch alignment); the LM tests are the stub above.            */
agc_status_t agc_job_systems_test(agc_job_ctx_t *ctx)
{
    if (!AGC_ASSERT(ctx != NULL))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    agc_status_t st = agc_phase_newmode((agc_sp_t) SYSTEMS_TEST_MODE);

    if (st != AGC_STATUS_OK)
    {
        return (st);
    }

    if (agc_state_vehicle() == AGC_VEHICLE_CSM)
    {
        return (agc_csm_prelaunch_imutest(ctx)); /* FALL INTO IMUTEST       */
    }

    if (!AGC_ASSERT(agc_state_vehicle() == AGC_VEHICLE_LM))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (AGC_STATUS_UNIMPLEMENTED);
}
