# status-handling -- rule 7: no (void)-cast agc_ calls in src/

## Scope
Remove every `(void) agc_...(...)` cast in src/ (712 sites, 711 flagged by
tools/check_rules.py + 1 split over two lines by clang-format) by either
making the callee `void` (cannot fail when called correctly) or doing what
the AGC does on the failure path at that site.

## Baseline (start)
* check_rules: 713 hard (711 rule 7, 1 rule 4 entry_step_a, 1 rule 1
  false positive in display_if.c), 148 warnings.
* 241 .c compile with -c; 7 test binaries link (core/numeric, core/state,
  core/state_lm, exec/exec, io/nouns, nav/navaux, oracle/core_numeric);
  core/state and nav/navaux each had 2 failing checks before this lane.
* make format-check clean; check_style 28787 findings.

## Progress
- TODO Phase A: void core/exec APIs + tools/status_void.cocci
- TODO Phase B: per-site failure handling

## Per-callee decisions
(filled in as done)

## Lessons
