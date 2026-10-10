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
  dsky/             PINBALL (display, keys, load, monitor, nvsub, scale,
                    sfin, state, verbs, internal.h), display_if.c, noun_*.c
  service/          t4rupt*.c (common, csm optics, lm radar), service.c,
                    rtb_ops.c, sysmisc_ext.h
  (telemetry/, verbs/ are not created until they hold >= 3 files: down
   telemetry, uplink update, extended verbs, self-check)
src/imu/            IMU moding, compensation, alignment, maneuvers
                    (files named without the imu_ prefix: mode, gyro,
                    compensation, vflags*, internal.h, pending.h)
src/nav/
  conic/            apsides, common, geom, kepler, lambert, timeradius,
                    timetheta, universal, internal.h
  integration/      init, jobs, flags_csm, setitctr_lm, orbital_integration,
                    svdwn, internal.h, pending.h
  auxiliary/        latlong, planetary, measurement, free_fall, ephemerides,
                    star_tables, util, internal.h
src/guidance/       powered-flight subroutines, shared burn programs
                    (external_delta_v, burn_steering), servicer_common
src/programs/
  navigation/       shared P20 core: common, w_matrix_monitor,
                    ground_track_determination, noun_data, flags_{csm,lm},
                    glue, internal.h, pending.h (job-id shim)
  targeting/        csi, csi_cdh_targeting, tpi, tpi_tpm_targeting,
                    stable_orbit, target_delta_v, initvel,
                    lambert_aimpoint_guidance_lm (LM only; lm/targeting/
                    would hold < 3 files)
  displays/         orbit_parameter_display, rendezvous_parameter_display,
                    out_of_plane_display, display
  alignment/        common, locsam, orientation, planet, imu_realign.c,
                    sighting_data_and_gyro_torquing.c
src/csm/            Comanche-only
  entry/            lexicon, reentry, programs, CM body attitude, dap
  dap/rcs/          RCS DAP, jet selection, FDAI needles, DAP data load
  dap/tvc/          TVC init/executive/mass properties/restarts/pitch-yaw/
                    stroke test/roll DAP
  dap/              mysubs.c, state.c (shared; < 3 files so no common/)
  burns/            sps/, earth_orbit_insertion, servicer, external_delta_v
    return_to_earth/ program, conic, velocity, precision, internal.h,
                    pending.h
  navigation/       P20-P25 CSM side: navigation, orbital_navigation,
                    cislunar_navigation, landmark_navigation,
                    tracking_attitude, tracking_data, tpi_search
  optics/           sxtmark, s_band_antenna, geometry
  prelaunch_align.c (< 3 files in prelaunch/)
src/lm/             Luminary-only
  navigation/       P20-P25 LM side: navigation, tracking_attitude,
                    tracking_data, measurement_update
  radar/            rendezvous radar: designate, moding, read, leadin,
                    acquisition, powered_flight_designate, radar.h
  landing/          guidance, displays, aborts, state, throttle,
                    findcduw, lunar_landing
  burns/            dps_burn*, ignition, servicer*
  dap/rcs/          idler, P-axis, Q/R-axis, TJET law, rate/Kalman, T6RUPT,
                    needles, rcsmon (failure monitor), spsrcs (docked law)
  dap/              shared glue: dapif.c, state.c, arith.c, accs.c and
                    accont.c (1/ACCS, 1/ACCONT), gts.c (trim gimbal),
                    internal.h, pending.h
  ascent.c, s_band_antenna.c, lem_geometry.c  (ascent/, optics/ not yet
                    >= 3 files)
src/sim/            host harness: clock, interrupts, DSKY console (no lib)
tests/              unit tests, mirroring src/ (core exec nav io imu guidance
                    programs csm lm oracle); one binary per file found
                    recursively, -DAGC_HOST_DEBUG
tools/              check_rules.py (Power of Ten lint)
docs/modules/       working notes per agent/module
```

Layering (lower may not call higher):
core < exec < io/imu/nav < guidance < programs < csm|lm < sim.

Dispatch: `src/exec/dispatch.c` holds the one `switch` from `agc_job_id_t`
/ `agc_task_id_t` to the job/task functions, so the call graph stays
static and recursion-free (rules 1 and 9).

File-name rule: inside a folder a file drops any prefix that only repeats an
enclosing folder name (conic/kepler.c, not conic/conic_kepler.c); AGC names
and _csm/_lm suffixes in shared folders are kept.

Subfolder rule: a src/ subfolder exists only when it holds >= 3 files;
otherwise its files stay in the parent. A private header included from a
different folder is promoted to include/agc/ with an `agc_` prefix, so no
moved file needs a `../` include (the one pre-existing cross-folder include,
csm/prelaunch_align.c -> ../imu/imu_pending.h, is untouched).

## Folder rules (2026-10-10)

* A file's folder path says where it belongs; its name says what it does.
  No folder-name prefixes on files, no `agc_` prefix on .c files.
* Three or more files sharing a prefix in one folder form a subfolder
  named by the prefix (burns/sps/, dsky/pinball/, dsky/nouns/, core/interp/).
* Shared glue stays at the parent level; never a `common/` folder.
* A subfolder needs at least three files.
* No adapter layers. Files named glue, shim, ext, pending, adapter or
  wrapper are scaffolding from parallel translation and are deleted at
  integration. A module calls another only through that module's public
  header, with the real function names. (mysubs.c is NOT such a file: MYSUBS
  is a real AGC log section.)

## Header layout (2026-10-10)

```
include/agc/
  core/       status.h assert.h word.h interp.h constants.h state.h channels.h
              flags_common.h flags_csm.h flags_lm.h
  exec/       exec.h jobs.h            (pending.h: cross-lane shim, removed at integration)
  io/         dsky.h nouns.h           (service_internal.h: to be folded into service.h)
  imu/        imu.h
  nav/        conic.h integration.h auxiliary.h
  guidance/   guidance.h               (pending.h: cross-lane shim)
  programs/   rendezvous.h alignment.h (alignment_internal.h, alignment_pending.h: shims)
  csm/        entry.h entry_lexicon.h dap.h (entry_internal.h, entry_pending.h: shims)
  lm/         landing.h dap.h
```

* Include as a path: `#include "agc/nav/conic.h"`. The `agc/` directory is
  the namespace; file names carry no prefix.
* Public headers mirror the src/ subsystem tree.
* A header used by one lane only lives beside its code as `internal.h` /
  `pending.h` at the lane's parent folder (src/csm/dap/internal.h) and is
  included relatively (`"../internal.h"`).
* `*_pending.h` and `*_internal.h` under include/agc/ are temporary
  cross-lane shims; integration folds their declarations into the real API
  headers and deletes them.
