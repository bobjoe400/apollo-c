/*
 * main.c -- minimal host driver (integration stub; the real harness with
 * clock, interrupts and DSKY console is a later lane).
 *
 * Usage: agc_sim [csm|lm] [centiseconds]
 *
 * Selects the vehicle, does the machine fresh start (DOFSTART) and runs
 * the software clock for the given time (default 1000 cs): every
 * centisecond TIME1/TIME2 advance, the waitlist counter TIME3 advances
 * (T3RUPT when it overflows), and the executive runs every ready job.
 * A software restart requested by BAILOUT / POODOO / ABORT is serviced
 * between jobs, as the AGC's ENEMA would be. At the end it prints the
 * major mode and the alarm registers.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc/core/assert.h"
#include "agc/core/state.h"
#include "agc/exec/exec.h"

#define SIM_MAX_CS      360000L  /* one hour of AGC time               */
#define SIM_JOBS_PER_CS 16       /* bound on the executive per tick     */
#define SIM_DEFAULT_CS  1000L

static long sim_jobs;  /* jobs run                                    */
static long sim_rests; /* software restarts serviced                  */

/* Run the executive until it is idle (DUMMYJOB) or the bound is hit.    */
static agc_status_t sim_run_jobs(void)
{
    agc_status_t last = AGC_STATUS_OK;

    for (int i = 0; i < SIM_JOBS_PER_CS; i++)
    {
        if (agc_exec_restart_pending())
        {
            last = agc_exec_service_restart();
            sim_rests++;
        }

        agc_status_t st = agc_exec_run_one();

        if (st == AGC_STATUS_NOT_FOUND)
        {
            break; /* idle */
        }

        sim_jobs++;
        last = st;
    }

    if (!AGC_ASSERT(sim_jobs >= 0))
    {
        return (AGC_STATUS_BAD_ARG);
    }

    return (last);
}

static agc_vehicle_t sim_vehicle(int argc, char *argv[])
{
    if (argc > 1 && strcmp(argv[1], "lm") == 0)
    {
        return (AGC_VEHICLE_LM);
    }

    return (AGC_VEHICLE_CSM);
}

static long sim_duration(int argc, char *argv[])
{
    long cs = SIM_DEFAULT_CS;

    if (argc > 2)
    {
        cs = strtol(argv[2], NULL, 10);
    }

    if (cs < 1 || cs > SIM_MAX_CS)
    {
        cs = SIM_DEFAULT_CS;
    }

    return (cs);
}

int main(int argc, char *argv[])
{
    agc_vehicle_t v  = sim_vehicle(argc, argv);
    long          cs = sim_duration(argc, argv);

    agc_state_init(v);
    agc_status_t st = agc_exec_init();

    if (st == AGC_STATUS_OK)
    {
        st = agc_waitlist_init();
    }

    if (st == AGC_STATUS_OK)
    {
        st = agc_fresh_dofstart(); /* DOFSTART: machine fresh start */
    }

    for (long t = 0; t < cs && st != AGC_STATUS_BAD_ARG; t++)
    {
        agc_time_advance_cs(1);
        agc_inhint();
        st = agc_waitlist_tick(1); /* TIME3; T3RUPT on overflow */
        agc_relint();
        agc_status_t sj = sim_run_jobs();

        st = (st == AGC_STATUS_OK) ? sj : st;
    }

    /* fprintf's result is irrelevant: the run is over (rule 7 note). */
    (void) printf("agc_sim %s: %ld cs, %ld jobs, %ld restarts, MODREG %d, "
                  "FAILREG %05o %05o %05o, last status %d\n",
                  (v == AGC_VEHICLE_LM) ? "LM" : "CSM", cs, sim_jobs,
                  sim_rests, (int) agc_erase_modreg(),
                  (unsigned) agc_erase_failreg(0),
                  (unsigned) agc_erase_failreg(1),
                  (unsigned) agc_erase_failreg(2), (int) st);

    return ((st == AGC_STATUS_BAD_ARG) ? 1 : 0);
}
