# Assignment: verify-collapse   (tier: review; Opus)

Independent verification of the status-handling-2 pass (commits
1b72892..a02850c; `git diff 1b72892 a02850c -- src include tests`). That
lane deleted ~2,700 guards, converted ~60 to status propagation, inlined
216 single-caller helpers, removed or rewrote 162 returns, and made ~20
functions void, on its own judgment. Build, 34 test binaries (~9,600
oracle checks against yaAGC) and 7 sim scenarios are green, so arithmetic
on tested inputs is intact. Nobody else has read the decisions. You do.

You are a second pair of eyes, not a style pass. Do not reformat, rename
or restructure. Edit src/ only to repair a change you have shown to be
wrong (restore a guard as a fallible status check with a caller that
handles it, or correct the merged control flow), and add a test that
would have caught it. Keep out of src/sim/ and sim/scenarios/ (the sim
lane is running there). docs/CODING_STANDARD.md rules apply to anything
you write; `make format` on files you edit.

## What to check, in this order

1. **Guard deletions with side effects or mixed conditions.** The lane
   found two failure classes itself: a deleted condition that carried the
   only call (`!AGC_ASSERT(decode(code, &c))`), and a condition mixing an
   assertion with normal flow (`!AGC_ASSERT(x) || n == 0`). Over the whole
   diff, list every removed `AGC_ASSERT(...)` whose argument contains a
   call with an `&` argument or an assignment, or whose `if` had another
   disjunct/conjunct; confirm each call survives and each normal-flow
   branch is still taken. tools/prove/manual.tsv has the hand decisions
   with reasons; build/prove/tree is the pre-run snapshot;
   `git diff 1b72892 a02850c` is the ground truth.
2. **Deletions in code no test reaches.** build/prove/eva_verdicts.tsv
   marks DEAD the sites no entry point reached. A deleted guard in a
   function that is DEAD in every CSV has no test behind it: enumerate
   those functions (join the diff's removed guards against the DEAD set
   by file+function) and read each deletion's reason against the code.
3. **manual.tsv sample.** Of the remaining GUARANTEED rows take at least
   150 spread across directories (and every row whose reason is a module
   invariant on a static: check all writers of that static with grep).
   Record agree / disagree per row; disagree means you show an input or
   state that reaches the site with the fact false.
4. **Status conversions.** For each function that gained an agc_status_t
   return (list in docs/modules/status-handling-2.md Counts and the
   final report in docs/WORKLOG.md), trace the new failure path to where
   the AGC's behaviour happens (BAILOUT/POODOO reaching the restart
   machinery, or the caller's own AGC-defined handling). A status that is
   returned but ignored by a caller, or compared with `!= BAD_ARG`
   instead of `== OK`, is a finding.
5. **Inlined helpers.** For the 216 merges (tools/inline_one.py rounds;
   the lane's note names js_acy/js_bdz by hand), check the ones whose
   helper had more than one exit or wrote a parameter: the merged body
   must take the same branches. Spot-check 40 of the single-exit ones.
6. **Return rewrites.** Sample 40 of the 147 if/else rewrites and all 15
   deleted trailing returns: no lost side effect, no changed condition.
7. **Void conversions and dropped parameters.** For the ~20 functions
   made void and the 190 helpers that lost ctx/ip/jc/p: confirm no caller
   depended on the old failure result and no dropped parameter was read.
8. **Test hygiene.** tests still count "assert reports" in two places
   (test_sysmisc 1 expected, test_integration 1 at
   src/nav/integration/init.c:196). A test must check the status a call
   returns, never an assertion report count; fix those two if they are in
   files no other lane is editing, otherwise record them.

## Output
- docs/modules/verify-collapse.md: Progress (checkpoint after every
  numbered item), Findings (file:line, what was changed, why it is wrong,
  what you did), Agree/disagree counts per item, Lessons for
  docs/TIPS.md. Row in docs/MODULE_STATUS.md.
- Fixes (if any) with a regression test, `make`, `make test`, `make
  check`, check_rules 0 hard green.
- Final report <= 15 lines: counts checked, findings, fixes, residual
  doubt.
