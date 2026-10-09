# ARCHITECTURE.md

```
include/agc/        public API (lead-owned contract)
  agc_status.h      status codes
  agc_assert.h      AGC_ASSERT
  agc_word.h        SP/DP/TP fixed-point model, SP sin/cos
  agc_interp.h      interpretive ops: scalar, vector, matrix, OVFIND
  agc_exec.h        executive, waitlist, alarms, phase tables
  agc_jobs.h        job/task id registry (integrated by lead)
  agc_channels.h    I/O channel image and bit names
  agc_state.h       shared erasables, flagwords, state vectors, time
  agc_flags_csm.h   Comanche flagword bit names   (state-flags agent)
  agc_flags_lm.h    Luminary flagword bit names   (state-flags agent)
  agc_dsky.h        DSKY/pinball interface        (io-pinball agent)
  agc_nav.h         conic/integration interfaces  (nav agents)
  agc_imu.h         IMU interfaces                (imu agent)
  agc_guid.h        guidance interfaces           (guidance agents)
src/core/           numeric model, state, channels      (libagc)
src/exec/           executive, waitlist, restarts, alarms, dispatch
src/io/             pinball/DSKY, keyrupt, T4RUPT, downlink, extended verbs
src/imu/            IMU moding, compensation, alignment, maneuvers
src/nav/            conic, integration, lat/long, planetary, ephemerides
src/guidance/       powered-flight subroutines, P30s, P40s shared parts
src/programs/       P20s rendezvous, P50s alignment, R-routines (shared)
src/csm/            Comanche-only: entry (P61-67), TVC/RCS DAP, P11, P37
src/lm/             Luminary-only: landing, ascent, LM DAP, AGS init
src/sim/            host harness: clock, interrupts, DSKY console (no lib)
tests/              unit tests (one binary per file, -DAGC_HOST_DEBUG)
tools/              check_rules.py (Power of Ten lint)
docs/modules/       working notes per agent/module
```

Layering (lower may not call higher):
core < exec < io/imu/nav < guidance < programs < csm|lm < sim.

Dispatch: `src/exec/dispatch.c` holds the one `switch` from `agc_job_id_t`
/ `agc_task_id_t` to the job/task functions, so the call graph stays
static and recursion-free (rules 1 and 9).
