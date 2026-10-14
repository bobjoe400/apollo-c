# Repository organisation: review, target layout, apply script

Organize lane, 2026-10-13. Index of every file: [INDEX.md](INDEX.md)
(`python3 -I tools/repo_index.py > docs/overview/INDEX.md`). Lane
note: [docs/modules/organize.md](../modules/organize.md).

This review covers two things: where files sit, and how the code is
structured inside them. Each finding falls into one of three groups:

* **Applied now** (docs, docs/agents, docs/overview, tools/, sim/ outside the
  NASSP lane): done in the working tree. `make`, `make test`, `make check`,
  `make nassp-replay` and `make lm-sim` pass on a copy of it.
* **Prepared** (src/, include/, tests/, the NASSP areas, and every change
  that does not alter what the code does): the script
  `tools/apply_layout.sh` with its data in `tools/layout/`. It was tested on
  a scratch copy of the tree and passed `make`, `make test`, `make check`,
  `make prolog-report` (0), `make always-ok-report` (0), `make nassp-replay`
  (5/5) and `make lm-sim` (34/34).
  The lead runs it between NASSP milestones (see "How to apply").
* **Findings only**: anything that would change behaviour, or that needs a
  reviewer to show the change is harmless. These are listed below with the
  exact sites. None of them is applied.

## 1. Criteria and verdicts

| # | Criterion | Before | After the moves applied now | After apply_layout.sh |
|---|---|---|---|---|
| 1 | Flight code by subsystem, harness by purpose, tools by task, history by date, without the lane notes | src/ by subsystem (good); harness drivers in tools/; history mixed into docs/ and docs/modules/ | sim/ holds every harness (driver and data together); docs/history/ holds the dated record; INDEX.md groups the tools by task | prelaunch/ subfolder; the NASSP files in two folders, each with one stated role |
| 2 | No finished files beside live ones | 41 finished assignments (with the resume note), 20 wave notes, 6 one-off coccinelle/generator files, wave output TSVs in overview/ | moved to docs/history/, tools/archive/, docs/overview/data/ | the script archives itself |
| 3 | Names say what a thing does (no lane names or dates) | `tests/io/sysmisc.c`, `tests/nav/navaux.c`, `tests/oracle/nav_integ.c`, `src/exec/erasable.c` (holds MODECADR), `src/io/dsky/display_if.c`, `src/lm/dap/dapif.c`, `src/guidance/guid_state.c` (repeats its folder), `src/nassp/nassp_engine.c`, `tools/nassp_edits.py` | `sim/lm_simulator/harness.tcl` | all renamed (section 3) |
| 4 | Operational docs apart from the process record | WORKLOG, RENAMES, AGENT_BRIEF and finished assignments next to STATE/PLAN | docs/history/ (record), docs/agents/ (live process only: `_common.md`, `brief.md`, `nassp-integration.md`, `review_items.md`) | none needed |
| 5 | Makefile targets grouped, documented, none dead | every target has a comment and none is dead; `make callgraph` was broken (it compiled src/nassp, which needs NASSP's headers; free_fall.c fails `-Werror` at -O0) | callgraph writes to docs/overview/data/; src/nassp excluded from CG_SRCS (M1, applied) | M2 (the -O0 warning in free_fall.c) is a flight-code cast, routed as a finding |
| 6 | Tests mirror src | yes at subsystem level. Six files carry lane or abbreviated names (sysmisc, navaux, nav_aux, nav_integ, align x2) | -- | renamed |
| 7 | One self-describing NASSP area | five places: src/nassp (shim + CMake + patches), sim/nassp (driver C, data, Lua, host scripts), tools/nassp_*.py, two docs | -- | two places, each with one role: **src/nassp/** = what NASSP's build compiles (`engine.c`, `CMakeLists.txt`; it must stay here because patch 0001 runs `add_subdirectory(${APOLLO_C_DIR}/src/nassp)` in the already-patched host trees); **sim/nassp/** = everything that runs against NASSP (patches/, replay.c + replay.py, mission_tables.py + mission_tolerances.tsv, checklist_to_lua.py, lua/, host/, images/). Docs stay in docs/ |

## 2. Applied now (working tree, uncommitted; lead commits)

* `docs/WORKLOG.md`, `docs/RENAMES.md`, `docs/RENAMES_inventory.txt` ->
  `docs/history/`. `docs/AGENT_BRIEF.md` -> `docs/agents/brief.md`.
* 41 finished assignments `docs/agents/<lane>.md` ->
  `docs/history/assignments/` (`_resume_after_naming.md` ->
  `resume_after_naming.md`). Kept live: `_common.md`, `brief.md`,
  `nassp-integration.md`, `review_items.md`.
* 20 tree-wide wave notes `docs/modules/{always-ok, collapse, document,
  finish, fix-b6, follow-up-done-lanes, format2, format3, integrate,
  layering, loose-ends, naming, prolog-consistency, restructure,
  status-handling, status-handling-2, style, style2, style-final,
  verify-collapse}.md` -> `docs/history/waves/`. docs/modules/ now holds only
  the subsystem lanes, their reviews, the harness lanes (sim, lm-simulator,
  nassp-integration, oracle, oracle-cases, closed-loop-fixes, p37) and
  organize.
* Generated tables `docs/overview/{callgraph.tsv, callgraph.dot,
  collapse_candidates.tsv, guards.tsv, one_liners.tsv,
  long_statements.tsv}` -> `docs/overview/data/`. Writers updated:
  Makefile `callgraph`, tools/callgraph.py, one_liners.py,
  prove/guards.py, prove/eta.py, inline_one.py, unwrap.py.
* Finished one-offs -> `tools/archive/`: `rename.cocci` and
  `gen_rename_cocci.py` (naming phase 2 applied, RENAMES ledger complete),
  `status_void.cocci`, `assert_recover.cocci` (status-handling),
  `cocci/setter_status.cocci`, `cocci/vehicle_guard.cocci` (finish). Kept
  live: `cocci/always_ok_void.cocci` with `drop_final_return.py`, which are
  the fix procedure behind the live `make always-ok-report` gate.
* Harness drivers -> sim/: `tools/sim_client.py` -> `sim/sim_client.py`,
  `tools/lm_sim.py` -> `sim/lm_simulator/lm_sim.py`,
  `tools/lm_sim_harness.tcl` -> `sim/lm_simulator/harness.tcl` (import
  path and ROOT updated).
* All references were rewritten (Makefile, README, docs, tool paths,
  relative Markdown links in the moved notes, uncrustify.cfg comments).
  Not edited: the NASSP lane's two docs, which reference none of the moved
  paths, and `src/sim/main.c:51` (a comment naming tools/sim_client.py).
  apply_layout.sh rewrites that comment.
* New: `tools/repo_index.py` (renders INDEX.md; rerun after any move),
  `docs/overview/data/structure/` (the per-item lists behind section 4), and
  a .gitignore entry for the NASSP shim's run logs (`apollo-c-agc-*.log`).
  These logs land in the repo root when the shim runs from there; one was
  committed and then removed in an earlier commit.
* docs/ARCHITECTURE.md: the layout block's tail now describes src/sim,
  src/nassp, src/erasable_map, tests, sim, tools and docs as they are.

## 3. Prepared: tools/apply_layout.sh

### How to apply

    git add -A && git commit ...            # clean tree; NASSP lane idle
    sh tools/apply_layout.sh --dry-run      # lists every step
    sh tools/apply_layout.sh [--lm-sim]     # applies, regenerates xref and the module map, runs the gates, archives itself

Every step is idempotent. The script stops if src, include, tests, sim,
tools or the Makefile have uncommitted changes. The lists were made on
69bce7f (with the NASSP lane's edits of that time) and touch no file of the NASSP lane's C code. If a lane changes
a listed file before the script runs, a row that no longer applies prints
SKIP, and a gate failure names the row to drop. tools/layout/analyze.py
regenerates the lists. After it runs, tell the
nassp-integration lane its new paths:

* `tools/nassp_*.py` -> `sim/nassp/{replay,mission_tables,checklist_to_lua}.py`
* `sim/nassp/edits.tsv` -> `sim/nassp/mission_tolerances.tsv`
* patches -> `sim/nassp/patches/`
* `src/nassp/nassp_engine.c` -> `src/nassp/engine.c`

src/nassp/CMakeLists.txt is updated by the script. The host trees need no
re-patch: patch 0001 still points at src/nassp. Update the host sync script
(outside the repository) for the moved patches if it copies them.

### Steps and counts (the scratch run)

| Step | Data | Count | Why it cannot change behaviour |
|---|---|---|---|
| Dead functions deleted | `tools/layout/dead_functions.tsv` | 10 | no caller in src/, tests/, the sim or the NASSP shim, and no AGC label: `agc_csm_prelaunch_ervector`, `agc_csm_prelaunch_thetan`, `exec_running_job_prio`, `agc_imu_pipa_interval`, `agc_kalc_outcome`, `agc_dsky_xyzreg`, `agc_extverb_verb`, `agc_ags_agsk`, `agc_lm_dap_time6_count` (its prolog says the sim calls it; the sim does not), `sim_socket_clients` |
| Prototypes never defined | `stale_prototypes.tsv` | 3 | `agc_csm_vtogain_body` (sps/internal.h), `agc_rendz_tracking_bvectors` (rendezvous.h), `lm_dap_paxis` (lm/dap/internal.h) |
| Public prototype removed, function made `static` | `make_static.tsv` | 102 | only its own file calls it; no test does; the compiler sees the same calls |
| Unused `#include` lines removed | `unused_includes.tsv` | 213 (in .c files) | for each line: gcc -O2 object code (objdump -d) is identical without it, and the debug build compiles |
| File moves / renames | `moves.tsv` | 22 (+3 rows already applied, kept so src/sim/main.c is rewritten) | git mv; #include spellings, Makefile, CMakeLists.txt, docs rewritten |
| Generated docs | xref.md, module map | 2 | regenerated from the moved tree |

### The git mv list (tools/layout/moves.tsv)

```
tools/nassp_replay.py                      sim/nassp/replay.py
tools/nassp_edits.py                       sim/nassp/mission_tables.py
tools/nassp_cl2lua.py                      sim/nassp/checklist_to_lua.py
sim/nassp/edits.tsv                        sim/nassp/mission_tolerances.tsv
src/nassp/patches/0001-apollo-c-agc-backend.patch     sim/nassp/patches/
src/nassp/patches/0002-lem-lua-test-interface.patch   sim/nassp/patches/
src/nassp/patches/0003-saturn-console-mode.patch      sim/nassp/patches/
src/nassp/nassp_engine.c                   src/nassp/engine.c
src/csm/prelaunch_align.c                  src/csm/prelaunch/align.c
src/csm/prelaunch_gyrocompass.c            src/csm/prelaunch/gyrocompass.c
src/csm/prelaunch_verify.c                 src/csm/prelaunch/verify.c
src/csm/prelaunch_internal.h               src/csm/prelaunch/internal.h
src/guidance/guid_state.c                  src/guidance/state.c
src/io/dsky/display_if.c                   src/io/dsky/display_interface.c
src/lm/dap/dapif.c                         src/lm/dap/interface.c
src/exec/erasable.c                        src/exec/modecadr.c
tests/nav/navaux.c                         tests/nav/auxiliary.c
tests/programs/align.c                     tests/programs/alignment.c
tests/io/sysmisc.c                         tests/io/service_verbs_telemetry.c
tests/oracle/nav_aux.c                     tests/oracle/nav_auxiliary.c
tests/oracle/nav_integ.c                   tests/oracle/nav_integration.c
tests/oracle/align.c                       tests/oracle/alignment.c
```

tools/registry.tsv keeps `csm` as the owning-folder group of the prelaunch
rows. That column orders the generated job ids, so changing it would
renumber them. It is a grouping label, not a path.

### Considered and not proposed

* **Splitting src/core/ into numeric/ and erasable/.** Every file in
  core/interp includes the private `mpac.h`, so the split would need either
  a cross-folder relative include, which the folder rules forbid, or
  promoting mpac.h to the public API. The gain is small.
* **Regrouping the live tools/ scripts into task subfolders.** Six scripts
  import `cgast` and compute ROOT from their own depth, and agents call
  `tools/check_rules.py` and `tools/format.sh` by path. The task grouping is
  in INDEX.md instead.
* **Moving `io/verbs/extended_verbs_lm_radar.c` to lm/radar/ and
  `programs/alignment/geometry_csm.c` to csm/optics/.** Both are
  vehicle-only, but `io/verbs/extended_verbs_lm.c` and
  `programs/displays/rendezvous_parameter_display.c` call into them, so the
  move would create upward calls.
* **Untracking the build artefacts in tools/oracle** (`build/*.o`, `bin/yaAGC`,
  `bin/yaYUL`, `ropes/*.symtab`; about 24 MB, listed in .gitignore but committed).
  The oracle generators need the symtabs and binaries without SRC, so
  dropping them is a policy decision for the lead.

## 4. Code structure findings (counts per category)

The analysis inputs were `make callgraph` (regenerated: 4,022 functions,
31,046 edges), `tools/xref.py` (3,075 labels), ctags for the headers, and
gcc for the include checks. The scripts are listed in
docs/modules/organize.md. The full per-item lists are in
[data/structure/](data/structure/): a_wrong_owner.tsv (with the verdict
column), b_header_internals.tsv, c_duplicates.tsv, d_dead.tsv, e_files.tsv,
f_vehicle.tsv and g_unused_includes.tsv. They were made at the pre-script
state of the tree.

### (a) Functions whose callers are all in another folder: 251 candidates, 0 moves

These are extern functions whose every caller is in one other folder. Each
was judged against the AGC section it translates and the static storage it
touches:

* 106 are accessors or helpers with no AGC label. They stay with the
  storage they read.
* 75 translate a section their own file translates. They stay.
* 56 translate a section neither folder owns (INTERPRETER helpers, shared
  utilities). They stay in the lower layer.
* 14 translate a section owned by the caller's folder. All 14 are restart
  and fresh-start resets (STARTSUB, SLAP1, DOFSTART pieces such as
  `agc_t4_startsub_reset`, `agc_telem_slap1`, `agc_selfcheck_restart`) that
  each module supplies to `exec/change_program` and that touch the module's
  own statics. Moving them would mean exporting those statics. **Keep.**

No function was moved. The real ownership problems are cross-vehicle ones,
listed under (f).

### (b) Header internals: 390 prototypes and 1,000 macros/types/enumerators checked

* **102 made static (script).**
* **34 AGC-labelled functions that nothing calls, not even their own
  file.** This is a finding: either a wiring gap or dead translation.
  `agc_rupt_t3`, `agc_lm_dap_t5rupt`, `agc_lm_dap_t6_pending` and
  `agc_phase_new` deserve a look first. The others:
  `agc_dp_div_ovf_only`, `agc_dp_mul_raw`, `agc_erase_sfail`,
  `agc_earth_orbit_insertion_monitor_set_padload`, `agc_abort_restart`,
  `agc_fresh_dofstart`, `agc_imu_comp_irig`, `agc_dsky_{jamproc, jamterm,
  nvmonopt, nvsbwait}`, `agc_rtb_{2v1sto2s, incrcdus, pulseimu, quality1,
  readpips, transp, vecsgnag}`, `agc_svc_{upent2, downent2}`,
  `agc_erase_stiksens`, `agc_lm_dap_needles_init`, `agc_ephem_csm_solpos`,
  `agc_planet_{earthmx, newangle}`, `agc_conic_iterator`,
  `agc_integ_othprec`, `agc_ags_buffer`, `agc_erase_trunbias`,
  `agc_rendz_rr_downlink`.
* 82 are used only by their own file and by tests. They are test seams;
  keep them public.
* **123 are used only inside one folder, with no test user.** Proposal: move
  each prototype to that folder's internal.h. The largest groups are
  io/verbs 36, lm/landing 20, csm/entry 17, exec 15, lm/alignment 10. This
  is not scripted, because each user file must include the folder's
  internal.h.
* Macros and types: 182 macros, 237 enumerators and 14 typedefs are used by
  a single .c file. Proposal: move them into that file. 163 macros and 152
  enumerators are used nowhere. These are mostly complete AGC tables (flag
  bits, channel bits, alarm codes) and are kept on purpose. The 98
  "unused" struct and enum tags are only the names behind typedefs.

### (c) Same AGC routine translated more than once: 144 cross-folder label rows

xref also lists every function whose prolog only *names* a label, so most
of the 144 rows are callers, not copies. Another 66 rows are per-vehicle
variants (csm/ and lm/), which is expected. After reading the rows, these
look like real duplicates. Each needs a behaviour comparison before the
callers are pointed at the owner's copy, so none is applied:

| AGC routine | Owner (keep) | Other copies |
|---|---|---|
| CDULOGIC | `agc_rtb_cdulogic` io/service/rtb_ops.c | `agc_imu_cdulogic` imu/cdu_trig.c, `agc_align_cdulogic` programs/alignment/common.c |
| 1STO2S / 1TO2SUB | `agc_rtb_1sto2s` rtb_ops.c | `agc_align_ones_to_twos` (test-only caller), `align_words_to_cdu` word_forms.c, `lm_dap_ones_to_twos` |
| CDUTRIG / CALCSMSC | `agc_imu_cdutrig`, `agc_imu_calcsmsc` imu/ | `agc_align_cdutrig`, `agc_align_calcsmsc` programs/alignment/common.c |
| LOADTIME | `agc_rtb_loadtime` | `agc_integ_loadtime` nav/integration/init.c, `align_loadtime` |
| SIGNMPAC | `agc_rtb_signmpac` | `entry_signmpac` csm/entry/reentry.c, `bovb_signmpac`/`store_signmpac` imu/maneuver_calc.c |
| QUALITY1/2 | `agc_rtb_quality1/2` (quality1 never called) | `lunar_j22_term` nav/integration/orbital_integration.c |
| LOG, EARROT1/2 (CSM_GEOMETRY) | `agc_csm_log` (only tests call it), `agc_csm_earrot1/2` programs/alignment/geometry_csm.c | `entry_log`, `entry_earrot` csm/entry/reentry.c |
| KEPPREP | `agc_conic_kepprep` nav/conic/kepler.c | `agc_integ_kepprep` nav/integration/orbital_integration.c |
| LUNPOS / LUNVEL | `agc_ephem_csm_lunpos/lunvel` nav/auxiliary/ephemerides.c | `lunpos`, `lunvel` (static) orbital_integration.c |
| SELECTMU, MUTABLE | `agc_rendz_selectmu` programs/navigation/common.c; `agc_conic_ctx_init_` nav/conic/common.c | `csi_selectmu`, `tpi_selectmu` (targeting); `rz_mutable_mu` |
| BVECTORS, PERIAPO, CSMSTORE/LEMSTORE | `rz_bvectors`, `agc_rendz_periapo`, `rz_precset` programs/navigation/common.c | `td_bvectors` csm/navigation/tracking_data.c; `tps_trial`, `tps_store` csm/navigation/tpi_search.c |
| LALOTORV | `agc_latlong_to_r` nav/auxiliary/latlong.c | `cn_landmark` csm/navigation/cislunar_navigation.c |
| INCORP2, INTEGRVS | `agc_incorp_update_w` nav/auxiliary/measurement.c; `agc_integ_integrvs` | `mu_incorp2` lm/navigation, `rz_incorp2`; `rpd_from_base`, `rz_integrvs` |
| DOWNFLAG, GENTRAN, SHOWSUM | `agc_svc_downflag`, `agc_svc_gentran` (test-only caller) io/service/service.c; `agc_selfcheck_showsum_start` | `agc_extverb_downflag`; `nc_mrkbuf_copy`, `to_mrkbuf1`, `agc_guid_copycycle`; `verb_show_banksum` |
| PIPASR | `agc_pipasr_noscale` core/counters.c | `agc_guid_pipasr` guidance/servicer_common.c |
| REDO5.5 | (one shared copy) | static `redo5_5` in csm/burns/servicer.c and lm/burns/servicer/average_g.c |

A pattern: the RTB op codes in io/service/rtb_ops.c were translated, but
seven of them have no caller, because other lanes coded the same op inline.
Route this table to a review lane as one item per row: compare the words
the copies produce (the oracle already covers most of these routines), then
redirect the callers.

### (d) Dead code: 88 extern functions with no caller outside tests

* 11 have no label and no caller at all. 10 are deleted by the script.
  `agc_assert_report` is reached through the AGC_ASSERT macro, so it stays.
* 17 have no label and only test callers. They are test seams; keep them:
  `agc_dp_to_double` (41 test uses), `agc_dsky_keys`, `agc_integ_ctx_init`,
  `agc_chan_set_signal`, `agc_flag_bit`, ...
* 39 have an AGC label and no caller, 34 of them listed under (b). 21 have
  a label and only test callers (for example `agc_entry_range_predict`,
  `agc_entry_huntest_predict`, `agc_return_to_earth_xt1lim`). They are kept
  as translations (Definition of done item 1). The list goes to a reviewer
  as wiring-gap candidates.

### (e) File shape: 59 files over 800 lines; 6 mix sections

These need a split or a rename. Splitting means moving functions between
files, so each is a proposal for the owning review lane:

| File | Lines | Problem | Proposal |
|---|---|---|---|
| src/programs/navigation/common.c | 1596 | six sections: rendezvous erasables, P34-35 subroutines (SELECTMU .. MIDGIM), INCORP2/INTEGRVS copies, P20 helpers | split into `rendezvous_erasables.c` (storage + accessors) and `rendezvous_subroutines.c` (P34-35 shared subroutines); drop the copies in (c) |
| src/csm/entry/reentry.c | 2387 | REENTRY_CONTROL plus private copies of CSM_GEOMETRY (LOG, EARROT) and SIGNMPAC | call the owners' copies, after which reentry.c holds one section |
| src/lm/systems_test.c | 2173 | IMU_PERFORMANCE_TESTS 2 and 4 plus SYSTEM_TEST_STANDARD_LEAD_INS, and it calls the CSM's IMUTEST (f) | `src/lm/systems_test/` split at the section boundaries; IMUTEST moves to a shared imu file |
| src/nav/integration/orbital_integration.c | 1716 | ORBITAL_INTEGRATION plus copies of RTB QUALITY1/2, LUNPOS/LUNVEL | drop the copies (c) |
| src/imu/maneuver.c | 1171 | KALCMANU_STEERING with GIMBAL_LOCK_AVOIDANCE and ANGLFIND pieces (maneuver_calc.c holds the rest) | split at the section boundaries if a reviewer agrees |
| src/programs/displays/display.c | 865 | display subroutines of four sections | name it `targeting_displays.c` |
| generic names | -- | `nav/conic/common.c`, `programs/alignment/common.c` (889), `programs/navigation/common.c`, `nav/auxiliary/util.c` say nothing about what they hold | rename to their content when they are split; `exec/erasable.c` -> `modecadr.c` is scripted |

Not counted as problems: `exec/dispatch.c` (generated, rule-4 exempt) and
`erasable_map/address_map.c` (the NASSP lane's file, one section).

### (f) The csm/, lm/ and shared split: 5 cross-vehicle defects, 53 one-vehicle routines in shared folders

Code in a vehicle folder that the other vehicle uses. Each item needs a
function moved together with the statics it uses:

1. `agc_job_servexit` (csm/burns/servicer.c) is called by lm/burns/dps_burn.c (3 sites). It is shared SERVEXIT and belongs in guidance/servicer_common.c.
2. `agc_csm_prelaunch_imutest` (csm/prelaunch_align.c) is called by lm/systems_test.c ("FALL INTO IMUTEST"). This is the shared IMU_PERFORMANCE_TESTS entry and belongs in imu/.
3. `agc_task_preread_lm` (lm/burns/servicer/average_g.c) is called by csm/burns/sps/burn.c through a vehicle switch. That file is shared burn code sitting in csm/.
4. `agc_job_rendezvous_navigation_lm` (lm/navigation) is called by csm/navigation/navigation.c (P20/P22 entry for both vehicles). The vehicle dispatch belongs in programs/navigation/.
5. `agc_job_sband_b5off` (csm/optics/s_band_antenna.c) is the registry handler for both vehicles. It belongs in the shared service routines.

The 53 one-vehicle routines in shared folders are mostly `_csm`/`_lm`
variants and LM radar-test erasables. ARCHITECTURE.md allows suffixed
vehicle variants there. Moving them up would create upward calls, as with
the two files under "Considered and not proposed".

### (g) Include graph: 0 cycles; 285 includes that use nothing from the header

* 221 are in .c files. 213 are removed by the script (verified one by one:
  identical object code). 8 are needed for transitive declarations and are
  kept.
* 64 are in headers (for example `flags_common.h` -> `state.h`,
  `csm/dap.h` -> `exec/exec.h`). This is a proposal: removing one means
  rebuilding every includer, and some are umbrella includes that callers
  rely on.

## 5. Other findings

* M1 (applied) `make callgraph` failed: `CG_SRCS` included src/nassp, whose
  shim needs NASSP's `agc_engine.h`. CG_SRCS now excludes it.
* M2 `make callgraph` at -O0: `src/nav/auxiliary/free_fall.c:443`
  `-Werror=conversion` (agc_dp_t -> short). GCC reports it only at -O0
  (`make` builds at -O2), so `make callgraph` still stops there until it is fixed. Route to nav-aux review. It is a cast in
  flight code, so it is not applied.
* sim/nassp/lua/common.lua:287 says "Compared by tools/edits_diff.py".
  That tool does not exist (it is now `make nassp-mission` /
  nassp_edits.py). This is the NASSP lane's file.
* docs/ARCHITECTURE.md: the top block still lists the pre-restructure
  header names (`agc_status.h`, `agc_word.h`, ...). The "Header layout"
  section below it is the current one. The line "ascent.c, s_band_antenna.c,
  lem_geometry.c (ascent/, optics/ not yet >= 3 files)" is stale, since
  lm/optics/ exists. Fix when the document lane next runs.
* docs/MODULE_STATUS.md rows for finished lanes still name pre-rename files
  (`agc_*.c`, `pending.h`). It is history in a live file. Proposal: add a
  "Files produced (at the time)" caption, or move the finished rows into
  docs/history/.
* docs/overview/xref.md is stale: 26 links point at files that no longer
  exist (`src/guidance/powered_flight.c`, `src/csm/optics/geometry.c`).
  `tools/xref.py` regenerates it, and apply_layout.sh step 6 does so. No
  other relative Markdown link in docs/ is broken after the moves.
