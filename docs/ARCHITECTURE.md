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
  agc_*_internal.h / agc_*_pending.h / agc_sysmisc.h
                    former lane-private headers shared by more than one src
                    subfolder, promoted here by the restructure (see
                    docs/modules/restructure.md)
src/core/           numeric model, state, channels      (libagc)
src/exec/           executive, waitlist, restarts, alarms, dispatch
src/io/             downlink_lists_{csm,lm}.c (telemetry; < 3 files so no subfolder)
  dsky/             pinball_*.c, display_if.c, noun_*.c, pinball_internal.h
  service/          t4rupt*.c (common, csm optics, lm radar), service.c,
                    rtb_ops.c, sysmisc_ext.h
  (telemetry/, verbs/ are not created until they hold >= 3 files: down
   telemetry, uplink update, extended verbs, self-check)
src/imu/            IMU moding, compensation, alignment, maneuvers
src/nav/
  conic/            conic_*.c, conic_internal.h
  integration/      integration_*.c, orbital_integration.c, svdwn.c
  aux/              latlong, planetary, measurement, free_fall, ephemerides,
                    star_tables, navaux_util
src/guidance/       powered-flight subroutines, shared burn programs
                    (external_delta_v, burn_steering), servicer_common
src/programs/
  rendezvous/       rendz_*.c
  alignment/        align_*.c, imu_realign.c, sighting_data_and_gyro_torquing.c
src/csm/            Comanche-only
  entry/            entry lexicon, reentry, entry programs, CM body attitude,
                    entry DAP
  dap/rcs/          RCS DAP, jet selection, FDAI needles, DAP data load
  dap/tvc/          TVC init/executive/mass properties/restarts/pitch-yaw/
                    stroke test/roll DAP
  dap/              mysubs.c, dap_state.c (shared; < 3 files so no common/)
  burns/            sps_burn*, earth_orbit_insertion, return_to_earth,
                    servicer_csm, external_delta_v_csm
  optics/           sxtmark, s_band_antenna_csm, csm_geometry
  prelaunch_align.c (< 3 files in prelaunch/)
src/lm/             Luminary-only
  landing/          landing guidance/displays/aborts/state, throttle,
                    findcduw, lunar_landing
  burns/            dps_burn*, ignition, servicer_lm*
  dap/rcs/          idler, P-axis, Q/R-axis, TJET law, rate/Kalman, T6RUPT,
                    needles
  dap/common/       dapif.c, state.c, arith.c
  dap/gts.c         trim gimbal system (< 3 files in gimbal/)
  ascent.c, s_band_antenna_lm.c, lem_geometry.c  (ascent/, optics/ not yet
                    >= 3 files)
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

Subfolder rule: a src/ subfolder exists only when it holds >= 3 files;
otherwise its files stay in the parent. A private header included from a
different folder is promoted to include/agc/ with an `agc_` prefix, so no
moved file needs a `../` include (the one pre-existing cross-folder include,
csm/prelaunch_align.c -> ../imu/imu_pending.h, is untouched).
