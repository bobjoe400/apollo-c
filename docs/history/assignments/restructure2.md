# Assignment: restructure2   (tier: mechanical)

Follow docs/ARCHITECTURE.md "Folder rules" and "Header layout". Only the
`style` agent is running (it edits .clang-format, tools/check_style.py,
docs/STYLE.md, docs/templates/, the Makefile's format/style targets); do
not touch those files except the one Makefile line in step 4.

Step 1 -- split src/programs/rendezvous (39 files) by subject. Move with
`git mv`; drop `_csm`/`_lm` suffixes once inside a vehicle folder; fix
every relative `#include` and the paths in docs (plain text), except
WORKLOG/RENAMES history files.

  src/programs/navigation/     navigation core shared by both vehicles:
      common.c (if it is P20 common code; else keep with its users),
      w_matrix_monitor.c, ground_track_determination.c, noun_data.c,
      internal.h, flags_csm.c + flags_lm.c (vehicle flag glue stays with
      the shared core since both are small), glue.c
  src/programs/targeting/      csi.c csi_cdh_targeting.c tpi.c
      tpi_tpm_targeting.c stable_orbit.c target_delta_v.c initvel.c
      lambert_aimpoint_guidance_lm.c -> lambert_aimpoint_guidance.c is LM
      only: put it in src/lm/targeting/ ONLY if >=3 LM-only targeting files
      exist; otherwise keep it here with the suffix
  src/programs/displays/       orbit_parameter_display.c
      rendezvous_parameter_display.c out_of_plane_display.c display.c
  src/csm/navigation/          cislunar_navigation_csm.c
      landmark_navigation_csm.c orbital_navigation_csm.c navigation_csm.c
      tracking_attitude_csm.c tracking_data_csm.c tpi_search_csm.c
      (suffix dropped: cislunar_navigation.c, landmark_navigation.c, ...)
  src/lm/navigation/           navigation_lm.c tracking_attitude_lm.c
      tracking_data_lm.c measurement_update_lm.c (suffix dropped)
  src/lm/radar/                rr_designate_lm.c rr_moding_lm.c
      radar_read_lm.c radar_leadin_lm.c acquisition_lm.c
      powered_flight_designate_lm.c radar_lm.h
      (-> designate.c moding.c read.c leadin.c acquisition.c
          powered_flight_designate.c radar.h)
  ext.h: it is the lane's pending-id shim. Move it to
      src/programs/navigation/pending.h and fix includers; integration
      deletes it later.
  If a file does not fit, place it by its content and list it in the note.
  include/agc/programs/rendezvous.h: leave in place for now (public API
  header rename is integration's job) but add a top comment listing the
  new source folders.

Step 2 -- src/csm/burns: `git mv` return_to_earth.c
  return_to_earth_conic.c return_to_earth_velocity.c
  return_to_earth_precision.c (+ any return_to_earth_* header) into
  src/csm/burns/return_to_earth/ as program.c conic.c velocity.c
  precision.c (and internal.h / pending.h). Fix includes.

Step 3 -- tests/: mirror src. Create tests/{core,exec,nav,io,imu,guidance,
  programs,csm,lm,oracle}/ and move each test beside its subject
  (test_core_numeric.c -> tests/core/numeric.c, test_state*.c ->
  tests/core/, test_exec.c -> tests/exec/exec.c, test_conic.c ->
  tests/nav/conic.c, test_integration.c, test_navaux.c -> tests/nav/,
  test_dsky.c test_nouns.c test_sysmisc.c -> tests/io/, test_imu.c ->
  tests/imu/, test_guidance.c -> tests/guidance/, test_align.c
  test_rendezvous.c -> tests/programs/, test_csm_dap.c test_entry.c
  test_return_to_earth.c -> tests/csm/, test_landing.c test_lm_dap.c ->
  tests/lm/, oracle_*.c -> tests/oracle/). Drop the `test_` prefix. Fix
  the relative includes some tests use (e.g. ../src/exec/internal.h ->
  ../../src/exec/internal.h).

Step 4 -- Makefile: change ONLY the two test globs so tests are found
  recursively and binaries keep a unique name:
    TEST_SRCS := $(shell find tests -name '*.c' | sort)
    TEST_BINS := $(patsubst tests/%.c,$(BUILD)/tests/%,$(TEST_SRCS))
  and make the test pattern rule mkdir its subdirectory (it already does
  `@mkdir -p $(dir $@)`). Touch nothing else in the Makefile.

Step 5 -- verify: compile every .c in src/ and tests/ with the brief's
  flags (-c); compare against the list before you started (record it) --
  no new failures. `python3 -I tools/check_rules.py src include` unchanged
  hard-violation count. Update docs/ARCHITECTURE.md layout block and
  docs/modules/restructure.md (append a "Pass 2" move table). Report
  <= 12 lines.
