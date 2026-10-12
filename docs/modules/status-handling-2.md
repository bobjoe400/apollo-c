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
- TODO step 7: refined collapse protection (label entered from >= 2 places) + inline
- TODO step 8: trailing returns removed; short-tail early returns -> if/else
- TODO make format, make/test/scenarios, then `make prove` once on the
  finished tree and `make prove-classify`
