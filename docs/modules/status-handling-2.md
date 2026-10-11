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
- TODO EVA harness and guards.tsv
- TODO classification
- TODO deletion of guaranteed guards
- TODO conversion of fallible guards
- TODO one-liners
- TODO make prove, docs
