# status-handling-2 -- rule 7: no silent or sentinel failures

## Scope
docs/agents/status-handling-2.md, run alone over the whole tree.
Baseline (2026-10-11, commit 1b72892): `tools/check_rules.py src include`
reports 755 silent failures (void function returns on a failed
AGC_ASSERT) and 1,535 sentinel failures (value function returns a
constant on a failed AGC_ASSERT), 2,290 guards in all, plus 27 other
hard violations ((void) AGC_ASSERT / memset / sim printf casts).
Policy change during the run (user): rule 5 is no longer a quota; every
guard proved redundant is deleted without replacement; the density is
reported as information only.

## Method
1. Frama-C EVA (`make prove`, tools/prove/): AGC_ASSERT is replaced by a
   call whose ACSL precondition is the asserted condition
   (tools/prove/eva_assert.h); each test main is analysed in turn; the
   per-site verdicts (VALID / UNKNOWN / INVALID / unreached) are merged.
2. libclang structural classification of every guard EVA does not prove
   (call sites, argument shapes, fixpoint over pointer parameters).
3. GUARANTEED guards deleted; FALLIBLE ones converted to agc_status_t with
   out-parameters, callers updated.
4. One-line functions classified (PLAIN / NAMED / ACCESSOR).

## Progress
- DONE EVA harness: tools/prove/eva_assert.h (shim), tools/prove/eva_run.py
  (one analysis per entry point: sim + 34 test mains; snapshot tree;
  progress in tools/prove/progress.log, summary.txt, runs/<entry>.log),
  `make prove` (detached), `make prove-classify`. An exploratory run
  (precision 1, 7 at a time, ~40 min; sim, exec_exec, io_dsky timed out)
  is kept in build/prove_long. The full run was stopped by the lead:
  EVA runs ONCE on the finished tree (re-sequenced).
- DONE classifier tools/prove/guards.py (libclang index of src/ + tests/:
  call sites with established facts, static-variable writes, one-line
  predicates expanded, enum ranges, type maps; classify: parameter facts
  over every call site (fixpoint), module invariants, C-type ranges,
  word/enum type invariant, EVA VALID; DEFERRED lanes -> eva-valid-deferred;
  tools/prove/manual.tsv for hand decisions). tools/prove/apply.py deletes
  GUARANTEED rows (by line, or --match by function + condition).
- DONE baseline classification (build/prove/index_baseline.pkl; scratch
  guards_baseline_v1.tsv): of 2,285 void/value guards 1,309 GUARANTEED
  (576 structural, 733 EVA-only), and 1,513 of 3,094 status guards.
- DONE structural deletions: 1,756 guards (all kinds) in 207 files.
  Fallout: 190 job/helper functions lost their only use of ctx/ip/jc/p
  and dropped the parameter (job entries become f(void);
  tools/gen_registry.py emits `f()` for them and no longer repeats the
  public entry's checks in the per-vehicle switches); agc_dp_*_ovf_only
  keep the discarded MPAC in agc_interp_t.mpac; UNIT split into unit_core
  + agc_vec_unit_abval (FIGTIME read only LV). make, make test (34/34),
  make scenarios green after this step.
- DONE EVA-only deletions (exploratory run) in reviewed lanes: 823;
  242 + 12 re-inserted in the DEFERRED lanes (lead), 3 of those kept
  deleted by hand (postconditions of the loop/if just before,
  tools/prove/manual.tsv).
- DONE build fallout of the EVA-only step (make, make test, make scenarios
  green on resume; check_rules 1,015 hard: 413 silent + 575 sentinel + 27).
- Resume re-classification (guards.py index + classify on the current
  tree, scratch g1.tsv): 948 void/value guards FALLIBLE + 30 UNCALLED,
  no EVA verdicts apply (positions stale; prove runs once at the end).
  Hand decisions go to tools/prove/manual.tsv (file, function, condition,
  GUARANTEED, why) and are deleted with tools/prove/apply.py --match.
- IN PROGRESS hand pass over the FALLIBLE guards, directory by directory:
  - DONE src/core (95 guards): channel numbers/flag ids are program
    constants (DSKY channel nouns bounded in pin_chan_of, writability
    checked in pin_comp_write via new agc_chan_writable; sim pad loader
    rejects NaN and per-vehicle flagword counts); arithmetic postconditions
    and shift counts by construction; tests/core/state.c misuse calls
    replaced by predicate checks. check_rules 920.
  - DONE src/exec (142 guards): NEWJOB/INHINT/TIME3/phase-table module
    invariants, postconditions, constant arguments deleted; fresh-start
    chain (STARTSB2..STARTSUB, DOFSTART*, LIGHTSET, PTBAD, CSM GOPROG
    tail) returns agc_status_t and propagates (waitlist full -> BAILOUT);
    agc_exec_job_wake_if_asleep returns status (CCSHOLE), callers in
    nav/integration/init.c and imu/gyro.c propagate;
    exec_service_interp_alarm returns the POODOO 1302 outcome into the
    job/task status; agc_exec_core_info, agc_erase_set_failreg,
    exec_up_stoprate, exec_up_setcoars are void (guards guaranteed);
    agc_phase_change(group, code) -> agc_phase_change(code) (group is the
    code's GGG; spatch over 161 calls). check_rules 778.
  - DONE src/io/service (102 guards): T4RUPT chain returns status where
    the AGC can leave through BAILOUT (WAITLIST full: ISSZERO, PROCTNON,
    GLOCKMON, ISSUP, RRGIMON, RRAUTCHK; NOVAC PROCKEY; DSPOUT), with
    out-parameters for the TIME4 delay and the IMUMON/RRCDUCHK "go on"
    results; SETISSW, CAGESUB, CAGESUB2, agc_imu_clear_cmds void; 15-bit
    erasable/CDU words, store postconditions and vehicle-only accessors
    deleted. tests/io/sysmisc.c accepts BAILOUT from T4RUPTs run without
    servicing the waitlist. check_rules 676.
  - DONE src/io/dsky, io/telemetry, io/verbs (76 guards): pinball module
    invariants, postconditions and ranges by construction deleted;
    agc_dsky_set_dsptem, align_set_dsptem1, rz_set_dsptem_dp, agc_sv_get
    void (cannot fail; callers in csm/, lm/, programs/ updated);
    agc_extverb_chkpooh/ckmodcad return the ALM/END status with the
    test result in an out-parameter (9 callers return that status instead
    of OK); sublist_words returns the lookup status. check_rules 600;
    make scenarios green.
  - DONE src/imu (75 guards): cagetest returns the IMUBAD status (5 task
    callers return it); chklinus returns the BLANKET status;
    agc_rtb_v1sto2s, agc_imu_ballangs, agc_erase_set_modecadr void
    (callers in lm/, csm/ updated); the rest deleted (postconditions,
    15-bit words, module invariants, constant axes). check_rules 525.
  - DONE src/lm/dap (87, reviewed lane; its rule-5 quota guards; DV
    models a zero divisor), src/csm/entry (76; divisions go through DDV,
    which models a zero divisor), src/nav (62; ACCOMP/ATOPCSM failures
    now plain status tests into INTEG_NEXT_FAIL, OBLATE's mixed
    assert/branch condition split). check_rules 297.
  - DONE src/lm (115): STARTP66/STARTP67 return the NEWMODE (NOVAC)
    status into GUILDENSTERN; agc_rr_settrkf, agc_guid_cdu_nbsm,
    agc_imu_comp_get void (comp words checked at agc_imu_comp_load);
    agc_lm_dv's DV-by-zero is a modelled hardware case (plain if);
    the rest deleted. check_rules 178.
  - DONE src/csm (40), src/programs + src/guidance (64): agc_dsky_response
    and agc_ground_track_determination_flight_path void; the rest
    deleted. DONE src/sim (40): sim_channels_init and sim_channels_tick
    return status (start-up fails / g_stats.errors counts a refused CDU
    drive); input_channel reports a non-input channel (packet ignored);
    entry values are checked in sim_packet_parse / script_read.
  - DONE the 27 other hard violations: (void) AGC_ASSERT statements
    deleted (word_forms.c, geometry.c), stdlib (void) casts commented,
    sim_socket_wait's result used.
  - Mixed conditions (`!AGC_ASSERT(x) || normal`) are never deleted whole:
    the helper skips them and they were split by hand (sim, telemetry
    REFSMMAT bound, OBLATE, shifts); every deleted `st == OK` guard was
    audited against the diff and replaced by propagation or a void callee.
- DONE step 3/4 result: check_rules 0 hard violations (from 1,015 on
  resume, 2,317 at the start); make, make test (34/34), make scenarios
  (7 scenarios) green.
- DONE step 6, one-liners: tools/one_liners.py (libclang; state =
  file-scope/static non-const storage, transitively through callees) ->
  docs/overview/one_liners.tsv: 562 one-line functions, ACCESSOR 473,
  NAMED 89 (53 by AGC label, 36 by hand: word structure, instruction
  semantics, scale conversions, VN/flag coding), PLAIN 0. PLAIN ones
  removed: inlined agc_conic_value_ (41 calls -> agc_dp_from_words),
  pair_value (4 -> agc_dp_pack), pin_mag/pin_abs64 (15 -> llabs),
  dif_cont_valid (1); the range predicates agc_navaux_dp/vec/tp_ok and
  agc_integ_valid_ctx went with the guard conjuncts that used them (word
  type invariant / integration module invariant, 28 status guards
  edited); unused agc_w_ok, agc_tp_to_double, agc_sp_sub deleted. Rule
  and tool added to docs/CODING_STANDARD.md.
- DONE step 7, refined collapse protection: tools/callgraph.py counts the
  references of every AGC label in both ropes (operand tokens: TC, TCF,
  BZF, BZMF, CCS/INDEX tables, CADR/2CADR/ADRES, interpretive operands;
  the defining label field and comments excluded; the larger rope count)
  and protects a single-caller helper only when its ENTRY label (the
  first own label of its comment head; later ones are targets inside the
  routine) has >= 2 references (collapse_candidates.tsv protection
  `label<2refs:LABEL=n`, kept rows `label-entered:LABEL=n`).
  387 helpers lost their protection; 216 inlined: js_acy/js_bdz into
  js_roll by hand (ACYCHECK=0, BDZCHECK=0; the TAUCHCK early return
  became if/else), 214 by tools/inline_one.py in fixpoint rounds (before
  and after step 8); 4 (TLIM, ITURNON, IMUOP, IFAILJMP chain into
  IMUMON) restored as helpers: the merged IMUMON nested 9 blocks deep
  (Uncrustify aborts at that depth; inline_one.py now refuses a merge
  deeper than 5 blocks, "too-deep"). Kept with reason (172): 80
  multi-exit helpers (early returns that are error paths or carry a
  value: the collapse lane's rule), 41 multi-site, 34 too long for rule
  4, 17 other refusals (value-conditional, param-written, compile).
- DONE step 8, returns: tools/returns.py (libclang; `report`/`fix`):
  15 trailing `return;` deleted (their comments moved to the statement
  before); 147 normal-flow early returns rewritten as if/else (void
  functions, and value/status functions whose early return equals their
  closing constant return, e.g. `return (AGC_STATUS_OK)`), tail <= 12
  code lines, conditions negated plainly (`!(c)` only when needed).
  Early returns kept: error paths (status != OK, failed guards: 11
  void + every status error return), 46 with longer tails, 50 nested.
- DONE make format (318 files; format-check clean), make, make test
  (34/34), make scenarios (7), check_rules 0 hard.
- DONE one `make prove` on the finished tree (2026-10-11 19:05-20:05,
  3 at a time): 34 entry points OK, `sim` TIMEOUT at 60 min (agc_sim's
  own main runs unbounded; no CSV). Per-run sums: VALID 821, UNKNOWN 567,
  INVALID 4, DEAD 63,174; merged sites INVALID 3. Artefacts kept as
  produced: build/prove/*.csv, build/prove/eva_verdicts.tsv,
  build/prove/tree (snapshot, COMMIT), tools/prove/summary.txt,
  tools/prove/progress.log, tools/prove/runs/*.log (sim.log partial).
  A bounded headless proof entry for the sim was started and withdrawn
  on the revised direction (its files and run outputs removed, the run
  artefacts above restored as they were).
- DEFERRED (user direction): applying EVA VALID deletions and the
  eva-valid-deferred marks (`make prove-classify`, apply.py --eva-only)
  is the project-end cleanup, after all reviews and after the sim runs
  under the VirtualAGC GUI; EVA reruns then (with a bounded sim entry).
- DONE EVA findings recorded as bug leads in docs/agents/review_items.md
  ("EVA bug leads"), one line each under the owning lane. Fixed now
  (plain C defects): INITVEL read LAMBERT's out-struct before testing
  the status (initvel.c, uninitialised on a LAMBERT entry failure);
  LM ASCENT ENGOFF swallowed a WAITLIST failure (as_engoff now returns
  it, BAILOUT 1203). False alarms (no fix, reasons in review_items):
  the signed overflows at orbital_integration.c XSUM and divide.c GENDDV
  (bounded NORM counts / normalised divisor not carried through
  out-parameters), the uninitialised o.x2/o.t reads (non-relational:
  status OK <=> out written).
- DONE check_rules: the silent/sentinel check now reads the whole guard
  block (it read three lines, so `{ agc_relint(); return; }` passed; a
  block that raises the AGC's alarm/bailout/poodoo is not silent). It
  found 2 more: as_engoff (converted) and agc_imu_pipfree (postcondition
  of the INHINT just before, deleted).

## Counts (rule 7)
* Start of the lane (commit 1b72892): 2,290 silent/sentinel guards (755
  void + 1,535 value) + 27 other hard violations = 2,317.
* Resume: 1,015 hard (413 silent + 575 sentinel + 27).
* Hand pass after the resume: 990 rule-7 guards resolved (988 + the 2 the
  widened checker found): ~930 deleted as
  GUARANTEED (each row in tools/prove/manual.tsv with its reason: 998 rows
  now), ~60 converted (the guard was the visible end of a real failure:
  status returned and propagated, value moved behind a status/out-param or
  the callee made void); 27 other violations fixed. check_rules: 0 hard.
* Functions changed in signature (all callers updated, ~160 call sites
  plus tests): agc_exec_job_wake_if_asleep, exec_service_interp_alarm,
  sim_channels_init, sim_channels_tick, agc_extverb_chkpooh/ckmodcad
  (status + out-param); fresh-start chain STARTSB2..DOFSTART, LIGHTSET,
  PTBAD, CSM GOPROG tail, T4RUPT DSPOUT/PROCEEDE/ISSZERO/IMUCAGE/IFAILJMP/
  PROCTNON/RRGIMON, STARTP66/P67, CHKLINUS, CAGETEST, cdu_drive, LM
  as_engoff (to status); agc_phase_change (group argument dropped: it is the code's
  GGG); made void because they cannot fail when called correctly:
  agc_exec_core_info, agc_erase_set_failreg, agc_erase_set_modecadr,
  agc_dsky_set_dsptem, agc_dsky_response, agc_sv_get, agc_t4_setissw,
  agc_t4_cagesub, agc_t4_cagesub2, agc_imu_clear_cmds, agc_imu_comp_get,
  agc_rtb_v1sto2s, agc_imu_ballangs, agc_rr_settrkf, agc_guid_cdu_nbsm,
  agc_ground_track_determination_flight_path, align_set_dsptem1,
  rz_set_dsptem_dp, exec_up_stoprate, exec_up_setcoars.
* Entry checks added where a value enters: pin_chan_of (DSKY channel
  noun: 1..037), pin_comp_write (agc_chan_writable), sim pad loader (NaN
  and per-vehicle flagword count), sim input_channel (non-input channel
  ignored).

## Lessons
* Most "fallible" guards the classifier could not prove were module
  invariants (a static the module alone writes), postconditions of the
  statement just before, 15-bit erasable words, constant arguments, or
  arithmetic facts of the AGC routine. Reading the writes of the static
  settles them faster than any call-site analysis.
* The real failures hide in `st == OK` / `st != BAD_ARG` guards after a
  call: WAITLIST full (BAILOUT 1203), NOVAC with no core set (BAILOUT
  1202), CCSHOLE (POODOO 1103). Those must propagate; tolerating
  "anything but BAD_ARG" ran on after a BAILOUT the AGC never returns
  from. A test that pumps T4RUPT without servicing the waitlist now sees
  BAILOUT, as the AGC would.
* Once a guard is gone a callee often returns OK on every path: make it
  void (rule 7 first bullet) and its callers' "st" plumbing disappears.
* Never delete a guard whose condition mixes an assertion with normal
  flow (`!AGC_ASSERT(x) || n == 0`, `|| fd < 0`): split it. The deletion
  helper now refuses mixed conditions; one slipped through before that
  check (the downlist REFSMMAT word bound) and was restored.
* A guard condition can carry the side effect (`!AGC_ASSERT(decode(code,
  &c))`): deleting it deletes the call. Grep deleted conditions for `&`
  arguments.
* Interpretive divisions go through DDV, which models a zero divisor as
  the AGC does; the LM DAP's DV likewise. A `!= 0` guard before them is
  not protecting C from undefined behaviour. A C `/` is (agc_lm_dv: the
  zero case is the hardware's, a plain if).
* Inlining chains can nest beyond what Uncrustify accepts (9 levels):
  inline_one.py refuses merges deeper than 5 blocks.
* The returns rewrite needs a plain negation (`x != 0`, not `!(x == 0)`):
  `->` is not an operator.
* EVA's "got status invalid" is relative to the analysis' precision: two
  of the three INVALID sites are invariants between two module statics
  (core set 0 free <=> NEWJOB -0) or a sleeper list EVA cannot see filled;
  read the calling context in runs/<entry>.log before calling it a bug.
  The uninitialised-read alarms after `if (st != OK) return` are the
  non-relational domain's, except where the out-struct is read BEFORE the
  status test (INITVEL: a real one).
* The unbounded harness main (argv options, server loop) is not an EVA
  entry point; a bounded proof main is needed (project end).
