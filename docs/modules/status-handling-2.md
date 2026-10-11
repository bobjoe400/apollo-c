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
- IN PROGRESS build fallout of the EVA-only step; then FALLIBLE guards by
  hand, file group by file group (rows below)
- TODO conversion of fallible guards (status + out-parameters, callers)
- TODO one-liners (step 6)
- TODO step 7: refined collapse protection (label entered from >= 2 places) + inline
- TODO step 8: trailing returns removed; short-tail early returns -> if/else
- TODO make format, make/test/scenarios, then `make prove` once on the
  finished tree and `make prove-classify`
