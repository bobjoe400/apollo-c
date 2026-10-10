# restructure -- subsystem subfolders (2026-10-10)

## Scope
Moved .c files and lane-private .h files into subsystem subfolders per the assignment. include/agc/ stays flat (13 headers added, no existing header edited). Makefile unchanged (already globs src/ recursively). Verified: all 177 .c files under src/ and tests/ compile exactly as before (172 pass; the same 5 pre-existing failures: sxtmark, service, ignition, lunar_landing, imu_realign -- dangling job/accessor symbols, identical errors to baseline). tools/check_rules.py: 0 hard violations, 78 warnings (unchanged).

## Rules applied
* File names: inside a folder, a prefix that only repeats an enclosing folder name is dropped (conic_kepler.c -> conic/kepler.c, pinball_state.c -> dsky/state.c, imu_comp.c -> imu/compensation.c, rcs_dap.c -> rcs/t5rupt.c, integration_csm.c -> integration/flags_csm.c). AGC names (sxtmark, servicer, dps_burn, orbital_integration) and _csm/_lm suffixes in shared folders (io, nav/integration, programs/rendezvous, imu) are kept; _csm/_lm dropped inside csm/ and lm/ (servicer.c, s_band_antenna.c, geometry.c, guid_pending.h). Promoted include/agc headers keep their names.
* A subfolder needs >= 3 files (headers counted); otherwise files stay in the parent.
* A private header whose includer and target are in different folders after the move was promoted to include/agc/ as agc_<name>.h; includers now say `#include "agc/agc_<name>.h"`.
* A cross-folder include between two files that both did not move (csm/prelaunch_align.c -> ../imu/imu_pending.h; tests/test_exec.c -> ../src/exec/exec_internal.h) was left alone.

## Headers promoted to include/agc/
| Old | New | Reason |
|---|---|---|
| src/csm/dap/csm_dap_internal.h | include/agc/agc_csm_dap_internal.h | used by csm/dap, dap/rcs, dap/tvc |
| src/csm/dap/csm_dap_pending.h | include/agc/agc_csm_dap_pending.h | included by agc_csm_dap_internal.h |
| src/csm/entry_internal.h | include/agc/agc_entry_internal.h | csm/entry + csm/burns |
| src/csm/entry_lexicon.h | include/agc/agc_entry_lexicon.h | csm/entry + csm/burns |
| src/csm/entry_pending.h | include/agc/agc_entry_pending.h | csm/entry + csm/burns |
| src/exec/exec_pending.h | include/agc/agc_exec_pending.h | included by promoted entry/landing pending headers; exec files and tests/test_exec.c updated |
| src/guidance/guid_pending.h | include/agc/agc_guid_pending.h | src/guidance + csm/burns + lm/burns |
| src/io/sysmisc.h | include/agc/agc_sysmisc.h | io/service + lm + csm + io/downlink_lists |
| src/lm/dap/lm_dap_internal.h | include/agc/agc_lm_dap_internal.h | lm/dap/rcs, lm/dap/common, lm/dap/gts.c |
| src/lm/dap/lm_dap_pending.h | include/agc/agc_lm_dap_pending.h | included by agc_lm_dap_internal.h |
| src/lm/landing_pending.h | include/agc/agc_landing_pending.h | lm/landing + lm/ascent.c + lm/burns |
| src/programs/align_internal.h | include/agc/agc_align_internal.h | used by programs/alignment, csm/optics, lm |
| src/programs/align_pending.h | include/agc/agc_align_pending.h | included by agc_align_internal.h |

## Move table (old path -> final path; includes the folder-prefix renames)
| Old | New |
|---|---|
| src/csm/cm_body_attitude.c | src/csm/entry/cm_body_attitude.c |
| src/csm/csm_geometry.c | src/csm/optics/geometry.c |
| src/csm/dap/dap_state.c | src/csm/dap/state.c |
| src/csm/dap/jet_select.c | src/csm/dap/rcs/jet_select.c |
| src/csm/dap/needles.c | src/csm/dap/rcs/needles.c |
| src/csm/dap/rcs_attitude.c | src/csm/dap/rcs/attitude.c |
| src/csm/dap/rcs_dap.c | src/csm/dap/rcs/t5rupt.c |
| src/csm/dap/rcs_dap_exec.c | src/csm/dap/rcs/exec.c |
| src/csm/dap/rcs_dapdata.c | src/csm/dap/rcs/dapdata.c |
| src/csm/dap/rcs_filter.c | src/csm/dap/rcs/filter.c |
| src/csm/dap/rcs_manual.c | src/csm/dap/rcs/manual.c |
| src/csm/dap/tvc_daps.c | src/csm/dap/tvc/daps.c |
| src/csm/dap/tvc_exec.c | src/csm/dap/tvc/exec.c |
| src/csm/dap/tvc_init.c | src/csm/dap/tvc/init.c |
| src/csm/dap/tvc_massprop.c | src/csm/dap/tvc/massprop.c |
| src/csm/dap/tvc_restarts.c | src/csm/dap/tvc/restarts.c |
| src/csm/dap/tvc_rolldap.c | src/csm/dap/tvc/rolldap.c |
| src/csm/dap/tvc_stroke.c | src/csm/dap/tvc/stroke.c |
| src/csm/downlink_lists_csm.c | src/io/downlink_lists_csm.c |
| src/csm/earth_orbit_insertion.c | src/csm/burns/earth_orbit_insertion.c |
| src/csm/entry_dap.c | src/csm/entry/dap.c |
| src/csm/entry_lexicon.c | src/csm/entry/lexicon.c |
| src/csm/entry_programs.c | src/csm/entry/programs.c |
| src/csm/external_delta_v_csm.c | src/csm/burns/external_delta_v.c |
| src/csm/guid_pending_csm.h | src/csm/burns/guid_pending.h |
| src/csm/reentry.c | src/csm/entry/reentry.c |
| src/csm/return_to_earth.c | src/csm/burns/return_to_earth.c |
| src/csm/s_band_antenna_csm.c | src/csm/optics/s_band_antenna.c |
| src/csm/servicer_csm.c | src/csm/burns/servicer.c |
| src/csm/sps_burn.c | src/csm/burns/sps_burn.c |
| src/csm/sps_burn_clock.c | src/csm/burns/sps_burn_clock.c |
| src/csm/sps_burn_ignition.c | src/csm/burns/sps_burn_ignition.c |
| src/csm/sps_burn_internal.h | src/csm/burns/sps_burn_internal.h |
| src/csm/sps_burn_steer.c | src/csm/burns/sps_burn_steer.c |
| src/csm/sxtmark.c | src/csm/optics/sxtmark.c |
| src/csm/t4rupt_csm.c | src/io/service/t4rupt_csm.c |
| src/exec/exec_erasable.c | src/exec/erasable.c |
| src/exec/exec_internal.h | src/exec/internal.h |
| src/imu/imu_comp.c | src/imu/compensation.c |
| src/imu/imu_gyro.c | src/imu/gyro.c |
| src/imu/imu_internal.h | src/imu/internal.h |
| src/imu/imu_mode.c | src/imu/mode.c |
| src/imu/imu_pending.h | src/imu/pending.h |
| src/imu/imu_vflags.c | src/imu/vflags.c |
| src/imu/imu_vflags_csm.c | src/imu/vflags_csm.c |
| src/imu/imu_vflags_lm.c | src/imu/vflags_lm.c |
| src/io/display_if.c | src/io/dsky/display_if.c |
| src/io/noun_data.c | src/io/dsky/nouns/data.c |
| src/io/noun_scales.c | src/io/dsky/nouns/scales.c |
| src/io/noun_tables_csm.c | src/io/dsky/nouns/tables_csm.c |
| src/io/noun_tables_lm.c | src/io/dsky/nouns/tables_lm.c |
| src/io/pinball_display.c | src/io/dsky/pinball/display.c |
| src/io/pinball/internal.h | src/io/dsky/pinball/internal.h |
| src/io/pinball_keys.c | src/io/dsky/pinball/keys.c |
| src/io/pinball_load.c | src/io/dsky/pinball/load.c |
| src/io/pinball_monitor.c | src/io/dsky/pinball/monitor.c |
| src/io/pinball_nvsub.c | src/io/dsky/pinball/nvsub.c |
| src/io/pinball_scale.c | src/io/dsky/pinball/scale.c |
| src/io/pinball_sfin.c | src/io/dsky/pinball/sfin.c |
| src/io/pinball_state.c | src/io/dsky/pinball/state.c |
| src/io/pinball_verbs.c | src/io/dsky/pinball/verbs.c |
| src/io/rtb_ops.c | src/io/service/rtb_ops.c |
| src/io/service.c | src/io/service/service.c |
| src/io/sysmisc_ext.h | src/io/service/sysmisc_ext.h |
| src/io/t4rupt.c | src/io/service/t4rupt.c |
| src/lm/dap/arith.c | src/lm/dap/arith.c |
| src/lm/dap/dapidler.c | src/lm/dap/rcs/dapidler.c |
| src/lm/dap/dapif.c | src/lm/dap/dapif.c |
| src/lm/dap/needles.c | src/lm/dap/rcs/needles.c |
| src/lm/dap/paxis.c | src/lm/dap/rcs/paxis.c |
| src/lm/dap/pmanual.c | src/lm/dap/rcs/pmanual.c |
| src/lm/dap/qraxis.c | src/lm/dap/rcs/qraxis.c |
| src/lm/dap/qrjets.c | src/lm/dap/rcs/qrjets.c |
| src/lm/dap/qrmanual.c | src/lm/dap/rcs/qrmanual.c |
| src/lm/dap/rate.c | src/lm/dap/rcs/rate.c |
| src/lm/dap/state.c | src/lm/dap/state.c |
| src/lm/dap/t6rupt.c | src/lm/dap/rcs/t6rupt.c |
| src/lm/dap/tjetlaw.c | src/lm/dap/rcs/tjetlaw.c |
| src/lm/downlink_lists_lm.c | src/io/downlink_lists_lm.c |
| src/lm/dps_burn.c | src/lm/burns/dps_burn.c |
| src/lm/dps_burn_rasteer.c | src/lm/burns/dps_burn_rasteer.c |
| src/lm/findcduw.c | src/lm/landing/findcduw.c |
| src/lm/guid_pending_lm.h | src/lm/burns/guid_pending.h |
| src/lm/ignition.c | src/lm/burns/ignition.c |
| src/lm/ignition_internal.h | src/lm/burns/ignition_internal.h |
| src/lm/landing_aborts.c | src/lm/landing/aborts.c |
| src/lm/landing_displays.c | src/lm/landing/displays.c |
| src/lm/landing_guidance.c | src/lm/landing/guidance.c |
| src/lm/landing_state.c | src/lm/landing/state.c |
| src/lm/lunar_landing.c | src/lm/landing/lunar_landing.c |
| src/lm/s_band_antenna_lm.c | src/lm/s_band_antenna.c |
| src/lm/servicer_lm.c | src/lm/burns/servicer.c |
| src/lm/servicer_lm_internal.h | src/lm/burns/servicer_internal.h |
| src/lm/servicer_lm_lr.c | src/lm/burns/servicer_lr.c |
| src/lm/t4rupt_lm.c | src/io/service/t4rupt_lm.c |
| src/lm/throttle.c | src/lm/landing/throttle.c |
| src/nav/conic_apsides.c | src/nav/conic/apsides.c |
| src/nav/conic_common.c | src/nav/conic/common.c |
| src/nav/conic_geom.c | src/nav/conic/geom.c |
| src/nav/conic_internal.h | src/nav/conic/internal.h |
| src/nav/conic_kepler.c | src/nav/conic/kepler.c |
| src/nav/conic_lambert.c | src/nav/conic/lambert.c |
| src/nav/conic_timeradius.c | src/nav/conic/timeradius.c |
| src/nav/conic_timetheta.c | src/nav/conic/timetheta.c |
| src/nav/conic_universal.c | src/nav/conic/universal.c |
| src/nav/ephemerides.c | src/nav/auxiliary/ephemerides.c |
| src/nav/free_fall.c | src/nav/auxiliary/free_fall.c |
| src/nav/integ_internal.h | src/nav/integration/internal.h |
| src/nav/integ_pending.h | src/nav/integration/pending.h |
| src/nav/integration_csm.c | src/nav/integration/flags_csm.c |
| src/nav/integration_init.c | src/nav/integration/init.c |
| src/nav/integration_jobs.c | src/nav/integration/jobs.c |
| src/nav/integration_lm.c | src/nav/integration/setitctr_lm.c |
| src/nav/latlong.c | src/nav/auxiliary/latlong.c |
| src/nav/measurement.c | src/nav/auxiliary/measurement.c |
| src/nav/navaux_internal.h | src/nav/auxiliary/internal.h |
| src/nav/navaux_util.c | src/nav/auxiliary/util.c |
| src/nav/orbital_integration.c | src/nav/integration/orbital_integration.c |
| src/nav/planetary.c | src/nav/auxiliary/planetary.c |
| src/nav/star_tables.c | src/nav/auxiliary/star_tables.c |
| src/nav/svdwn.c | src/nav/integration/svdwn.c |
| src/programs/align_common.c | src/programs/alignment/common.c |
| src/programs/align_locsam.c | src/programs/alignment/locsam.c |
| src/programs/align_orientation.c | src/programs/alignment/orientation.c |
| src/programs/align_planet.c | src/programs/alignment/planet.c |
| src/programs/imu_realign.c | src/programs/alignment/imu_realign.c |
| src/programs/rendz_common.c | src/programs/rendezvous/common.c |
| src/programs/rendz_csi.c | src/programs/rendezvous/csi.c |
| src/programs/rendz_csi_cdh_targeting.c | src/programs/rendezvous/csi_cdh_targeting.c |
| src/programs/rendz_display.c | src/programs/rendezvous/display.c |
| src/programs/rendz_ext.h | src/programs/rendezvous/ext.h |
| src/programs/rendz_flags_csm.c | src/programs/rendezvous/flags_csm.c |
| src/programs/rendz_flags_lm.c | src/programs/rendezvous/flags_lm.c |
| src/programs/rendz_glue.c | src/programs/rendezvous/glue.c |
| src/programs/rendz_initvel.c | src/programs/rendezvous/initvel.c |
| src/programs/rendz_internal.h | src/programs/rendezvous/internal.h |
| src/programs/rendz_tpi.c | src/programs/rendezvous/tpi.c |
| src/programs/sighting_data_and_gyro_torquing.c | src/programs/alignment/sighting_data_and_gyro_torquing.c |

## Includes fixed
83 source/header/test files had `#include` lines rewritten: same-folder includes stay bare (`"x.h"`), promoted headers use `"agc/agc_x.h"`. tests/test_exec.c now includes agc/agc_exec_pending.h (exec_internal.h stays at ../src/exec/). Files that only had include changes without moving: src/exec/{fresh_start,phase_table,restart_tables,restarts,waitlist}.c, src/guidance/*.c, src/lm/{ascent,lem_geometry,s_band_antenna}.c, src/lm/dap/gts.c, src/csm/dap/{mysubs,state}.c.

## Placed by judgment
* src/io/downlink_lists_{csm,lm}.c: telemetry/ would hold only 2 files, so they sit in src/io/.
* src/io/service/t4rupt_{csm,lm}.c (from csm/ and lm/) and sysmisc_ext.h: assignment lists t4rupt*.c under io/service.
* src/csm/dap/{mysubs,state}.c: common/ would hold only 2 files once the two shared headers were promoted, so they stay in src/csm/dap/ (rcs/ and tvc/ are subfolders). dap/rcs gets needles.c (FDAI needles, RCS-CSM DAP text).
* src/csm/optics/geometry.c (sextant line of sight): placed with optics. src/lm/lem_geometry.c (RR gimbal angles) would be lm/optics but that folder has < 3 files, so it stays in src/lm/ with s_band_antenna_lm.c.
* src/lm/dap/gts.c: gimbal/ has 1 file, stays in src/lm/dap/. src/lm/ascent.c: ascent/ has 1 file, stays in src/lm/. src/csm/prelaunch_align.c: prelaunch/ has 1 file, stays in src/csm/.
* src/programs/alignment/{imu_realign.c,sighting_data_and_gyro_torquing.c}: P52 realign and R54/R55 sighting/gyro torquing, shared alignment programs.
* src/nav/svdwn.c -> nav/integration (per assignment). lm/landing_state.c -> lm/landing. csm/{servicer_csm,external_delta_v_csm}.c -> csm/burns.
* Not created (no files yet): io/verbs, io/telemetry, lm/tests, csm/prelaunch, lm/optics, lm/ascent, lm/dap/gimbal, csm/dap/common. Planned files named in older notes (extended_verbs, uplink_update, self_check, aotmark, ags_init, imu_tests) go to these folders once they reach 3 files.
* nav/aux was renamed to nav/auxiliary ("aux" is a reserved device name on Windows checkouts).

## Docs updated
docs/ARCHITECTURE.md layout block; path mentions in docs/*.md, docs/modules/*.md, docs/agents/*.md and in source comments (exact old-path -> new-path substitution, plus stale pre-naming file names such as p30.c, p37.c, sband_cm.c mapped to current files). Untouched as history: docs/WORKLOG.md, docs/RENAMES.md, docs/RENAMES_inventory.txt. Bare header names in prose (e.g. "landing_pending.h") were not rewritten; see the promotion table.
