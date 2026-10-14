# finish -- apply the EVA pass, close the silent-skip guards, final gates

## Scope
Lead's launch prompt (2026-10-12; no docs/agents/finish.md): the project finishes on the
EVA pass already on disk (build/prove/eva_verdicts.tsv, 33 completed entry
points: VALID 100, UNKNOWN 146, INVALID 2, DEAD 644; snapshot commit
2a1d608, identical to the tree this lane started from in src/include/tests).
No further EVA runs.

1. Delete the 100 VALID guards (manual.tsv rows), void the functions that
   are left always-OK and strip their callers' plumbing.
2. The 21 positive-assertion setter guards (`if (AGC_ASSERT(c)) { store; }`):
   VALID ones deleted under 1, the rest become fallible checks.
3. tools/check_rules.py: a bare run scans what `make check-rules` scans.
4. tools/prove/eva_run.py: the unbounded `sim` main is no longer an entry.
5. Gates.

## Design
| Step | Tool | Result |
|------|------|--------|
| VALID guards, plain `if (!AGC_ASSERT(..) [|| ..]) {..}` | tools/prove/apply.py --match (scratch tsv from eva_verdicts.tsv + CSV function names) | 96 deleted |
| VALID guards mixed with normal flow | by hand | 3 split, 1 statement deleted whole |
| always-OK fallout | tools/cocci/always_ok_void.cocci (6 waves, regex regenerated per wave), tools/drop_final_return.py, compiler-guided call sites, tools/gen_registry.py for void jobs | `make always-ok-report` 0 |
| setter guards | tools/archive/setter_status.cocci (definition + prototypes), callers by hand | 20 converted |
| prologs | scripted from `make prolog-report` (returns-void -> `RETURNS: none`), 6 stale BAD_ARG texts by hand | `make prolog-report` 0 |

## Progress
- DONE step 1: 100 VALID guards deleted, each with a tools/prove/manual.tsv
  row (class GUARANTEED, why `EVA VALID 2026-10-12 (<entries>)`). 96 by
  apply.py --match; by hand: executive.c end_of_job (`core[0].vac >= 0`
  stays; this was also one of the 21 setter guards), scale.c pin_comp_read
  and pin_comp_octal (`st != OK` stays), display_if.c
  agc_dsky_display_ecadr (both assertions of the statement VALID, so its
  condition was constant false: statement deleted). Every assertion on a
  VALID line was checked VALID in at least one run (two-assertion lines
  have two CSV rows).
- DONE fallout of step 1: unused buf_ok (gyro.c) deleted; restart_tables.c
  rst_table_lookup void (its only failure was the deleted index guard);
  parameters dropped: agc_dnlist_word_count's `v` (only the deleted
  vehicle assertion read it; tests/io/sysmisc.c updated by spatch),
  agc_job_dspmmjb's `ctx` (job entry f(void), dispatch regenerated);
  tools/gen_registry.py no longer emits the task-entry
  agc_interrupts_inhibited() guard (dispatch.c:2860 was VALID).
- DONE always-OK waves 1-6 (18, 12, 10, 5, 2, 2 functions) plus lightset
  and rst_table_lookup by hand: 51 functions void. Notable: the integration
  permanent-state accessors (agc_integ_perm_get/set, atopcsm/atoplem,
  ptoacsm), the pinball display chain (pin_dsptab_store, pin_dspin,
  pin_11dspin, pin_2blank, pin_dspdecvn, pin_dspoctwo, pin_updatvb,
  agc_job_dspmmjb), the fresh-start chain (startsb1/2, startsub, dofstart,
  phase_tables_ok, lightset, agc_fresh_dofstart, agc_fresh_start_slap1,
  exec_up_dofstart, exec_up_startsb2, agc_power_on, agc_waitlist_init,
  agc_lm_dap_fresh_start), the SXT/AOT mark helpers, agc_sv_set.
- DONE step 2: the 21 setter guards. 1 deleted (end_of_job, VALID). 20
  converted (all EVA DEAD or UNKNOWN): the 16 src/lm/dap/state.c setters
  (CDUXD.., DELCDUX.., OMEGAPD.., DELPEROR.., AOSQ/AOSR, STIKSENS,
  -RATEDB), agc_imu_set_gcompsw, agc_imu_set_pipa_interval,
  rz_set_csm_residuals: they return agc_status_t (BAD_ARG, nothing stored,
  when the value fails the assertion); csm/entry/dap.c dap_fdai: the
  SW/NDX range check moved before the SAVENDX store (fails with BAD_ARG,
  the index is then in range by construction). Every caller handles the
  status; void callers became status functions: maneuver.c set_cdud,
  loadcdud (+ new set_cdud_csm (void, CM), set_delcdu_lm, set_rate_lm),
  findcduw.c fc_rates, fc_transfer (FINDCDUW relints before returning
  the status), burnbaby.c ascent_ignition split into ascent_bias_load
  (P12IGN, status) and ascent_ignition (ABRTIGN, void),
  programs/alignment align_set1_pdt, align_ncoarse,
  drift_compensation_restart (their five job callers propagate). Tests:
  tests/imu/imu.c CHECKs, tests/oracle/imu.c oracle_cmp on the status.
  libclang call-site listing: no setter call is left as a bare statement.
- DONE step 3: check_rules.py with no argument scans DEFAULT_ROOTS (src,
  include, resolved against the repository, so the cwd does not matter);
  a missing directory is an error (exit 2), not an empty scan. The
  Makefile target now runs the bare form, so the two cannot drift.
- DONE step 4: eva_run.py entry_points() lists the test mains only; the
  docstring records why (14.4 GB at 78 % coverage on a 15 GB host after
  98 min, 2026-10-12) and that EVA is a one-time end-of-project cleanup
  here. CODING_STANDARD.md tooling (it did not name the sim entry) and
  the Makefile prove comment say the same.
- DONE step 5 gates: see Counts.

## Counts
* Guards deleted: 100 (96 apply.py, 4 by hand). Setter guards: 1 deleted
  (counted in the 100), 20 converted to fallible checks.
* Functions made void: 51. Functions newly returning agc_status_t: 19
  setters + set_cdud, loadcdud, fc_rates, fc_transfer, align_set1_pdt,
  align_ncoarse, drift_compensation_restart (+ 3 new helpers with status,
  1 void). Parameters dropped: 2. Helper deleted: 1 (buf_ok).
* INVALID kept (2): executive.c:314 spectest `newjob > 0` (the AGC's
  CCSHOLE, POODOO 1103) and restarts.c:106 start_job `prdt != 0 && job
  != NONE` -- both the AGC's own failure paths, reached by tests on
  purpose.
* Gates: make OK; make test 35 unit binaries 0 failures + 14 scenarios;
  make check OK (check-rules 0 hard, format-check clean, style report
  only, 416 findings tree-wide); make prolog-report 0; make
  always-ok-report 0; one-liners PLAIN 0 (docs/overview/data/one_liners.tsv
  regenerated). Style findings in the 110 touched files: 39 (40 before).

## Findings
* EVA's VALID is "valid on every path EVA reached". Two test misuse calls
  violate VALID guards because EVA never reached them (its analysis of
  those tests stopped propagating earlier): tests/io/sysmisc.c
  `agc_selfcheck_set_rope(NULL, 3) == BAD_ARG` (removed; the one real
  caller passes the static rope and AGC_ROPE_BANKS) and tests/io/nouns.c
  `agc_scale_sfconst(CSM, 20) == BAD_ARG` (replaced by code 19 == OK; the
  codes come from the const scale_defs table). Both surfaced only as test
  failures after the deletion: worth knowing before trusting a VALID on a
  public entry.
* `sim_harness` does not cover src/sim: eva_run analyses tests/sim/harness.c
  with lib_sources(), which leaves src/sim out, so eva_verdicts.tsv has no
  src/sim row at all. The src/sim assertions have no EVA verdict (recorded
  in the eva_run.py docstring; no rerun per the user's decision).
* Deleting the VALID guard inside a callee often removed the callee's only
  failure, and the always-OK cascade went 6 waves deep (pinball display,
  fresh start, integration state moves). always_ok.py only sees literal
  returns: lightset (returning a variable that is now always OK) had to be
  found by reading.

## Lessons
* A here-doc driven replace tool must strip the here-doc's final newline,
  or every last replacement ending mid-line splits that line and every
  one ending at a line end adds a blank line (caught by a diff scan for
  new double blanks / blank-before-brace and fixed).
* `st = (st == OK) ? f() : st;` chains need the project's `/* if OK */`
  comment (check_style ternary-comment).

## Program flow and decisions

_Documentation wave (2026-10-12), HEAD 99b2c6e._ A tree-wide pass (the EVA guard deletions and the final gates): it changed code owned by the translation lanes without changing what the program decides, so it has no program flow of its own. The flows as the code now stands are in the lane sections: [exec-sched](../../modules/exec-sched.md#program-flow-and-decisions), [core-numeric](../../modules/core-numeric.md#program-flow-and-decisions), [state-flags](../../modules/state-flags.md#program-flow-and-decisions), [nav-conic](../../modules/nav-conic.md#program-flow-and-decisions), [nav-integ](../../modules/nav-integ.md#program-flow-and-decisions), [nav-aux](../../modules/nav-aux.md#program-flow-and-decisions), [io-pinball](../../modules/io-pinball.md#program-flow-and-decisions), [io-nouns](../../modules/io-nouns.md#program-flow-and-decisions), [imu](../../modules/imu.md#program-flow-and-decisions), [guid-powered](../../modules/guid-powered.md#program-flow-and-decisions), [guid-landing](../../modules/guid-landing.md#program-flow-and-decisions), [guid-entry](../../modules/guid-entry.md#program-flow-and-decisions), [p37](../../modules/p37.md#program-flow-and-decisions), [dap-lm](../../modules/dap-lm.md#program-flow-and-decisions), [dap-csm](../../modules/dap-csm.md#program-flow-and-decisions), [prog-rendz](../../modules/prog-rendz.md#program-flow-and-decisions), [prog-align](../../modules/prog-align.md#program-flow-and-decisions), [sys-misc](../../modules/sys-misc.md#program-flow-and-decisions), [sim](../../modules/sim.md#program-flow-and-decisions); the system view is [docs/overview/README.md](../../overview/README.md) and its flowcharts.
