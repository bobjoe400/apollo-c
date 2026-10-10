# restructure -- subsystem subfolders (2026-10-10)

## Scope
Moved .c files and lane-private .h files into subsystem subfolders per the assignment. include/agc/ stays flat (13 headers added, no existing header edited). Makefile unchanged (already globs src/ recursively). Verified: all 177 .c files under src/ and tests/ compile exactly as before (172 pass; the same 5 pre-existing failures: sxtmark, service, ignition, lunar_landing, imu_realign -- dangling job/accessor symbols, identical errors to baseline). tools/check_rules.py: 0 hard violations, 78 warnings (unchanged).

## Rules applied
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

## Move table
| Old | New |
|---|---|
| src/csm/cm_body_attitude.c | src/csm/entry/cm_body_attitude.c |
| src/csm/csm_geometry.c | src/csm/optics/csm_geometry.c |
| src/csm/dap/jet_select.c | src/csm/dap/rcs/jet_select.c |
| src/csm/dap/needles.c | src/csm/dap/rcs/needles.c |
| src/csm/dap/rcs_attitude.c | src/csm/dap/rcs/rcs_attitude.c |
| src/csm/dap/rcs_dap.c | src/csm/dap/rcs/rcs_dap.c |
| src/csm/dap/rcs_dap_exec.c | src/csm/dap/rcs/rcs_dap_exec.c |
| src/csm/dap/rcs_dapdata.c | src/csm/dap/rcs/rcs_dapdata.c |
| src/csm/dap/rcs_filter.c | src/csm/dap/rcs/rcs_filter.c |
| src/csm/dap/rcs_manual.c | src/csm/dap/rcs/rcs_manual.c |
| src/csm/dap/tvc_daps.c | src/csm/dap/tvc/tvc_daps.c |
| src/csm/dap/tvc_exec.c | src/csm/dap/tvc/tvc_exec.c |
| src/csm/dap/tvc_init.c | src/csm/dap/tvc/tvc_init.c |
| src/csm/dap/tvc_massprop.c | src/csm/dap/tvc/tvc_massprop.c |
| src/csm/dap/tvc_restarts.c | src/csm/dap/tvc/tvc_restarts.c |
| src/csm/dap/tvc_rolldap.c | src/csm/dap/tvc/tvc_rolldap.c |
| src/csm/dap/tvc_stroke.c | src/csm/dap/tvc/tvc_stroke.c |
| src/csm/downlink_lists_csm.c | src/io/downlink_lists_csm.c |
| src/csm/earth_orbit_insertion.c | src/csm/burns/earth_orbit_insertion.c |
| src/csm/entry_dap.c | src/csm/entry/entry_dap.c |
| src/csm/entry_lexicon.c | src/csm/entry/entry_lexicon.c |
| src/csm/entry_programs.c | src/csm/entry/entry_programs.c |
| src/csm/external_delta_v_csm.c | src/csm/burns/external_delta_v_csm.c |
| src/csm/guid_pending_csm.h | src/csm/burns/guid_pending_csm.h |
| src/csm/reentry.c | src/csm/entry/reentry.c |
| src/csm/return_to_earth.c | src/csm/burns/return_to_earth.c |
| src/csm/s_band_antenna_csm.c | src/csm/optics/s_band_antenna_csm.c |
| src/csm/servicer_csm.c | src/csm/burns/servicer_csm.c |
| src/csm/sps_burn.c | src/csm/burns/sps_burn.c |
| src/csm/sps_burn_clock.c | src/csm/burns/sps_burn_clock.c |
| src/csm/sps_burn_ignition.c | src/csm/burns/sps_burn_ignition.c |
| src/csm/sps_burn_internal.h | src/csm/burns/sps_burn_internal.h |
| src/csm/sps_burn_steer.c | src/csm/burns/sps_burn_steer.c |
| src/csm/sxtmark.c | src/csm/optics/sxtmark.c |
| src/csm/t4rupt_csm.c | src/io/service/t4rupt_csm.c |
| src/io/display_if.c | src/io/dsky/display_if.c |
| src/io/noun_data.c | src/io/dsky/noun_data.c |
| src/io/noun_scales.c | src/io/dsky/noun_scales.c |
| src/io/noun_tables_csm.c | src/io/dsky/noun_tables_csm.c |
| src/io/noun_tables_lm.c | src/io/dsky/noun_tables_lm.c |
| src/io/pinball_display.c | src/io/dsky/pinball_display.c |
| src/io/pinball_internal.h | src/io/dsky/pinball_internal.h |
| src/io/pinball_keys.c | src/io/dsky/pinball_keys.c |
| src/io/pinball_load.c | src/io/dsky/pinball_load.c |
| src/io/pinball_monitor.c | src/io/dsky/pinball_monitor.c |
| src/io/pinball_nvsub.c | src/io/dsky/pinball_nvsub.c |
| src/io/pinball_scale.c | src/io/dsky/pinball_scale.c |
| src/io/pinball_sfin.c | src/io/dsky/pinball_sfin.c |
| src/io/pinball_state.c | src/io/dsky/pinball_state.c |
| src/io/pinball_verbs.c | src/io/dsky/pinball_verbs.c |
| src/io/rtb_ops.c | src/io/service/rtb_ops.c |
| src/io/service.c | src/io/service/service.c |
| src/io/sysmisc_ext.h | src/io/service/sysmisc_ext.h |
| src/io/t4rupt.c | src/io/service/t4rupt.c |
| src/lm/dap/arith.c | src/lm/dap/common/arith.c |
| src/lm/dap/dapidler.c | src/lm/dap/rcs/dapidler.c |
| src/lm/dap/dapif.c | src/lm/dap/common/dapif.c |
| src/lm/dap/needles.c | src/lm/dap/rcs/needles.c |
| src/lm/dap/paxis.c | src/lm/dap/rcs/paxis.c |
| src/lm/dap/pmanual.c | src/lm/dap/rcs/pmanual.c |
| src/lm/dap/qraxis.c | src/lm/dap/rcs/qraxis.c |
| src/lm/dap/qrjets.c | src/lm/dap/rcs/qrjets.c |
| src/lm/dap/qrmanual.c | src/lm/dap/rcs/qrmanual.c |
| src/lm/dap/rate.c | src/lm/dap/rcs/rate.c |
| src/lm/dap/state.c | src/lm/dap/common/state.c |
| src/lm/dap/t6rupt.c | src/lm/dap/rcs/t6rupt.c |
| src/lm/dap/tjetlaw.c | src/lm/dap/rcs/tjetlaw.c |
| src/lm/downlink_lists_lm.c | src/io/downlink_lists_lm.c |
| src/lm/dps_burn.c | src/lm/burns/dps_burn.c |
| src/lm/dps_burn_rasteer.c | src/lm/burns/dps_burn_rasteer.c |
| src/lm/findcduw.c | src/lm/landing/findcduw.c |
| src/lm/guid_pending_lm.h | src/lm/burns/guid_pending_lm.h |
| src/lm/ignition.c | src/lm/burns/ignition.c |
| src/lm/ignition_internal.h | src/lm/burns/ignition_internal.h |
| src/lm/landing_aborts.c | src/lm/landing/landing_aborts.c |
| src/lm/landing_displays.c | src/lm/landing/landing_displays.c |
| src/lm/landing_guidance.c | src/lm/landing/landing_guidance.c |
| src/lm/landing_state.c | src/lm/landing/landing_state.c |
| src/lm/lunar_landing.c | src/lm/landing/lunar_landing.c |
| src/lm/servicer_lm.c | src/lm/burns/servicer_lm.c |
| src/lm/servicer_lm_internal.h | src/lm/burns/servicer_lm_internal.h |
| src/lm/servicer_lm_lr.c | src/lm/burns/servicer_lm_lr.c |
| src/lm/t4rupt_lm.c | src/io/service/t4rupt_lm.c |
| src/lm/throttle.c | src/lm/landing/throttle.c |
| src/nav/conic_apsides.c | src/nav/conic/conic_apsides.c |
| src/nav/conic_common.c | src/nav/conic/conic_common.c |
| src/nav/conic_geom.c | src/nav/conic/conic_geom.c |
| src/nav/conic_internal.h | src/nav/conic/conic_internal.h |
| src/nav/conic_kepler.c | src/nav/conic/conic_kepler.c |
| src/nav/conic_lambert.c | src/nav/conic/conic_lambert.c |
| src/nav/conic_timeradius.c | src/nav/conic/conic_timeradius.c |
| src/nav/conic_timetheta.c | src/nav/conic/conic_timetheta.c |
| src/nav/conic_universal.c | src/nav/conic/conic_universal.c |
| src/nav/ephemerides.c | src/nav/aux/ephemerides.c |
| src/nav/free_fall.c | src/nav/aux/free_fall.c |
| src/nav/integ_internal.h | src/nav/integration/integ_internal.h |
| src/nav/integ_pending.h | src/nav/integration/integ_pending.h |
| src/nav/integration_csm.c | src/nav/integration/integration_csm.c |
| src/nav/integration_init.c | src/nav/integration/integration_init.c |
| src/nav/integration_jobs.c | src/nav/integration/integration_jobs.c |
| src/nav/integration_lm.c | src/nav/integration/integration_lm.c |
| src/nav/latlong.c | src/nav/aux/latlong.c |
| src/nav/measurement.c | src/nav/aux/measurement.c |
| src/nav/navaux_internal.h | src/nav/aux/navaux_internal.h |
| src/nav/navaux_util.c | src/nav/aux/navaux_util.c |
| src/nav/orbital_integration.c | src/nav/integration/orbital_integration.c |
| src/nav/planetary.c | src/nav/aux/planetary.c |
| src/nav/star_tables.c | src/nav/aux/star_tables.c |
| src/nav/svdwn.c | src/nav/integration/svdwn.c |
| src/programs/align_common.c | src/programs/alignment/align_common.c |
| src/programs/align_locsam.c | src/programs/alignment/align_locsam.c |
| src/programs/align_orientation.c | src/programs/alignment/align_orientation.c |
| src/programs/align_planet.c | src/programs/alignment/align_planet.c |
| src/programs/imu_realign.c | src/programs/alignment/imu_realign.c |
| src/programs/rendz_common.c | src/programs/rendezvous/rendz_common.c |
| src/programs/rendz_csi.c | src/programs/rendezvous/rendz_csi.c |
| src/programs/rendz_csi_cdh_targeting.c | src/programs/rendezvous/rendz_csi_cdh_targeting.c |
| src/programs/rendz_display.c | src/programs/rendezvous/rendz_display.c |
| src/programs/rendz_ext.h | src/programs/rendezvous/rendz_ext.h |
| src/programs/rendz_flags_csm.c | src/programs/rendezvous/rendz_flags_csm.c |
| src/programs/rendz_flags_lm.c | src/programs/rendezvous/rendz_flags_lm.c |
| src/programs/rendz_glue.c | src/programs/rendezvous/rendz_glue.c |
| src/programs/rendz_initvel.c | src/programs/rendezvous/rendz_initvel.c |
| src/programs/rendz_internal.h | src/programs/rendezvous/rendz_internal.h |
| src/programs/rendz_tpi.c | src/programs/rendezvous/rendz_tpi.c |
| src/programs/sighting_data_and_gyro_torquing.c | src/programs/alignment/sighting_data_and_gyro_torquing.c |

## Includes fixed
83 source/header/test files had `#include` lines rewritten: same-folder includes stay bare (`"x.h"`), promoted headers use `"agc/agc_x.h"`. tests/test_exec.c now includes agc/agc_exec_pending.h (exec_internal.h stays at ../src/exec/). Files that only had include changes without moving: src/exec/{fresh_start,phase_table,restart_tables,restarts,waitlist}.c, src/guidance/*.c, src/lm/{ascent,lem_geometry,s_band_antenna_lm}.c, src/lm/dap/gts.c, src/csm/dap/{mysubs,dap_state}.c.

## Placed by judgment
* src/io/downlink_lists_{csm,lm}.c: telemetry/ would hold only 2 files, so they sit in src/io/.
* src/io/service/t4rupt_{csm,lm}.c (from csm/ and lm/) and sysmisc_ext.h: assignment lists t4rupt*.c under io/service.
* src/csm/dap/{mysubs,dap_state}.c: common/ would hold only 2 files once the two shared headers were promoted, so they stay in src/csm/dap/ (rcs/ and tvc/ are subfolders). dap/rcs gets needles.c (FDAI needles, RCS-CSM DAP text).
* src/csm/optics/csm_geometry.c (sextant line of sight): placed with optics. src/lm/lem_geometry.c (RR gimbal angles) would be lm/optics but that folder has < 3 files, so it stays in src/lm/ with s_band_antenna_lm.c.
* src/lm/dap/gts.c: gimbal/ has 1 file, stays in src/lm/dap/. src/lm/ascent.c: ascent/ has 1 file, stays in src/lm/. src/csm/prelaunch_align.c: prelaunch/ has 1 file, stays in src/csm/.
* src/programs/alignment/{imu_realign.c,sighting_data_and_gyro_torquing.c}: P52 realign and R54/R55 sighting/gyro torquing, shared alignment programs.
* src/nav/svdwn.c -> nav/integration (per assignment). lm/landing_state.c -> lm/landing. csm/{servicer_csm,external_delta_v_csm}.c -> csm/burns.
* Not created (no files yet): io/verbs, io/telemetry, lm/tests, csm/prelaunch, lm/optics, lm/ascent, lm/dap/gimbal, csm/dap/common. Planned files named in older notes (extended_verbs, uplink_update, self_check, aotmark, ags_init, imu_tests) go to these folders once they reach 3 files.
* nav/aux: the directory name is as specified, but "aux" is a reserved device name on Windows checkouts; consider renaming to nav/navaux if that matters.

## Docs updated
docs/ARCHITECTURE.md layout block; path mentions in docs/*.md, docs/modules/*.md, docs/agents/*.md and in source comments (exact old-path -> new-path substitution, plus stale pre-naming file names such as p30.c, p37.c, sband_cm.c mapped to current files). Untouched as history: docs/WORKLOG.md, docs/RENAMES.md, docs/RENAMES_inventory.txt. Bare header names in prose (e.g. "landing_pending.h") were not rewritten; see the promotion table.
